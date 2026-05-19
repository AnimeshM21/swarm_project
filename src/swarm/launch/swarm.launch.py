"""
Unified swarm launch file.

Usage:
  ros2 launch swarm swarm.launch.py                         # 3 bots, 0 drones (default)
  ros2 launch swarm swarm.launch.py num_bots:=2 num_drones:=1
  ros2 launch swarm swarm.launch.py num_bots:=0 num_drones:=1  # drone only
"""

import os
import yaml
import atexit
import tempfile
import numpy as np
from math import sqrt

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, AppendEnvironmentVariable,
    DeclareLaunchArgument, GroupAction, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue


# ──────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────

def _is_valid_position(new_pos, existing, min_dist):
    """Euclidean distance check in XY for collision-free spawn."""
    return all(
        sqrt((new_pos[0] - p[0]) ** 2 + (new_pos[1] - p[1]) ** 2) >= min_dist
        for p in existing
    )


def _sample_positions(count, existing, rng, bounds=3.0, min_dist=1.5, max_attempts=500):
    """Return *count* XY positions that satisfy min_dist from each other and *existing*."""
    positions = []
    for _ in range(count):
        for attempt in range(max_attempts):
            pos = [rng.uniform(-bounds, bounds), rng.uniform(-bounds, bounds)]
            if _is_valid_position(pos, existing + positions, min_dist):
                positions.append(pos)
                break
        else:
            raise RuntimeError(
                f"Could not place {count} agents (min_dist={min_dist}) "
                f"in [{-bounds}, {bounds}] after {max_attempts} attempts per agent."
            )
    return positions


def _generate_bridge_yaml(num_bots, num_drones):
    """Build the list-of-dicts that ros_gz_bridge expects as config_file."""
    entries = [
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'ignition.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        }
    ]

    # ── Ground bots ──────────────────────────────────────────
    for i in range(1, num_bots + 1):
        ros_ns  = f'/bot_{i}'
        gz_model = f'/model/ground_bot_{i}'

        entries += [
            {'ros_topic_name': f'{ros_ns}/cmd_vel',
             'gz_topic_name':  f'{gz_model}/cmd_vel',
             'ros_type_name':  'geometry_msgs/msg/Twist',
             'gz_type_name':   'ignition.msgs.Twist',
             'direction':      'ROS_TO_GZ'},

            {'ros_topic_name': f'{ros_ns}/odom',
             'gz_topic_name':  f'{gz_model}/odometry',
             'ros_type_name':  'nav_msgs/msg/Odometry',
             'gz_type_name':   'ignition.msgs.Odometry',
             'direction':      'GZ_TO_ROS'},

            {'ros_topic_name': f'{ros_ns}/joint_states',
             'gz_topic_name':  f'{gz_model}/joint_states',
             'ros_type_name':  'sensor_msgs/msg/JointState',
             'gz_type_name':   'ignition.msgs.Model',
             'direction':      'GZ_TO_ROS'},

            {'ros_topic_name': f'{ros_ns}/imu',
             'gz_topic_name':  f'{gz_model}/imu',
             'ros_type_name':  'sensor_msgs/msg/Imu',
             'gz_type_name':   'ignition.msgs.IMU',
             'direction':      'GZ_TO_ROS'},

            {'ros_topic_name': f'{ros_ns}/camera/image_raw',
             'gz_topic_name':  f'{gz_model}/depth_camera/image',
             'ros_type_name':  'sensor_msgs/msg/Image',
             'gz_type_name':   'ignition.msgs.Image',
             'direction':      'GZ_TO_ROS'},

            {'ros_topic_name': f'{ros_ns}/lidar/points',
             'gz_topic_name':  f'{gz_model}/lidar/scan/points',
             'ros_type_name':  'sensor_msgs/msg/PointCloud2',
             'gz_type_name':   'ignition.msgs.PointCloudPacked',
             'direction':      'GZ_TO_ROS'},
        ]

    # ── Drones ───────────────────────────────────────────────
    for i in range(1, num_drones + 1):
        # Single-drone keeps the familiar '/drone' namespace.
        ros_ns = 'drone' if num_drones == 1 else f'drone_{i}'
        gz_ns  = ros_ns  # robotNamespace in the plugin matches

        entries += [
            {'ros_topic_name': f'/{ros_ns}/cmd_vel',
             'gz_topic_name':  f'/{gz_ns}/gazebo/command/twist',
             'ros_type_name':  'geometry_msgs/msg/Twist',
             'gz_type_name':   'ignition.msgs.Twist',
             'direction':      'ROS_TO_GZ'},

            {'ros_topic_name': f'/{ros_ns}/enable',
             'gz_topic_name':  f'/{gz_ns}/enable',
             'ros_type_name':  'std_msgs/msg/Bool',
             'gz_type_name':   'ignition.msgs.Boolean',
             'direction':      'ROS_TO_GZ'},

            {'ros_topic_name': f'/{ros_ns}/motor_speed_cmd',
             'gz_topic_name':  f'/{gz_ns}/gazebo/command/motor_speed',
             'ros_type_name':  'actuation_msgs/msg/Actuators',
             'gz_type_name':   'ignition.msgs.Actuators',
             'direction':      'ROS_TO_GZ'},
        ]
        for m in range(4):
            entries.append({
                'ros_topic_name': f'/{ros_ns}/motor_speed/{m}',
                'gz_topic_name':  f'/{gz_ns}/motor_speed/{m}',
                'ros_type_name':  'std_msgs/msg/Float64',
                'gz_type_name':   'ignition.msgs.Double',
                'direction':      'GZ_TO_ROS',
            })

    return entries


# ──────────────────────────────────────────────────────────────
# OpaqueFunction — runs at launch-time with resolved args
# ──────────────────────────────────────────────────────────────

def _launch_setup(context, *args, **kwargs):
    num_bots   = int(LaunchConfiguration('num_bots').perform(context))
    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    pkg_share   = get_package_share_directory('swarm')
    world_file  = os.path.join(pkg_share, 'world', 'swarm_world.sdf')
    bot_xacro   = os.path.join(pkg_share, 'model', 'robot.xacro')
    drone_xacro = os.path.join(pkg_share, 'model', 'robot_drone.xacro')

    actions = []

    # ── Write bridge YAML to a tempfile ──────────────────────
    bridge_entries = _generate_bridge_yaml(num_bots, num_drones)
    bridge_fd, bridge_path = tempfile.mkstemp(prefix='swarm_bridge_', suffix='.yaml')
    with os.fdopen(bridge_fd, 'w') as f:
        yaml.dump(bridge_entries, f, default_flow_style=False)
    atexit.register(os.unlink, bridge_path)  # clean up on process exit

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='swarm_bridge',
        output='screen',
        parameters=[{'config_file': bridge_path, 'use_sim_time': True}],
    ))

    # ── Spawn ground bots ────────────────────────────────────
    rng = np.random.default_rng(seed=42)
    all_positions = []

    bot_positions = _sample_positions(num_bots, all_positions, rng) if num_bots > 0 else []
    all_positions += bot_positions

    for i, pos in enumerate(bot_positions, start=1):
        ns         = f'bot_{i}'
        model_name = f'ground_bot_{i}'

        robot_desc = ParameterValue(
            Command(['xacro ', bot_xacro, ' frame_prefix:=', f'{ns}/']),
            value_type=str,
        )

        actions.append(GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', model_name,
                    '-topic', 'robot_description',
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', '0.1',
                ],
            ),
        ]))

    # ── Spawn drones ─────────────────────────────────────────
    drone_positions = _sample_positions(num_drones, all_positions, rng) if num_drones > 0 else []

    for i, pos in enumerate(drone_positions, start=1):
        if num_drones == 1:
            ns, model_name, robot_ns = 'drone', 'drone', 'drone'
            frame_prefix = ''
        else:
            ns = f'drone_{i}'
            model_name = ns
            robot_ns   = ns
            frame_prefix = f'{ns}/'

        robot_desc = ParameterValue(
            Command([
                'xacro ', drone_xacro,
                ' frame_prefix:=', frame_prefix,
                ' robot_ns:=', robot_ns,
            ]),
            value_type=str,
        )

        actions.append(GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', model_name,
                    '-topic', 'robot_description',
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', '2.0',
                ],
            ),
        ]))

    # ── Swarm teleop node ────────────────────────────────────
    actions.append(Node(
        package='swarm',
        executable='swarm_teleop',
        name='swarm_teleop',
        output='screen',
        parameters=[{
            'num_bots':   num_bots,
            'num_drones': num_drones,
            'use_sim_time': True,
        }],
    ))

    return actions


# ──────────────────────────────────────────────────────────────
# Top-level generate_launch_description
# ──────────────────────────────────────────────────────────────

def generate_launch_description():
    pkg_share = get_package_share_directory('swarm')

    resource_paths = [
        os.path.join(pkg_share, 'model'),
        os.path.join(pkg_share, 'world'),
        os.path.dirname(pkg_share),
    ]

    return LaunchDescription([
        # ── Launch arguments ──
        DeclareLaunchArgument('num_bots',   default_value='3', description='Number of ground bots'),
        DeclareLaunchArgument('num_drones', default_value='0', description='Number of drones'),

        # ── Gazebo resource paths (both vars for Fortress compatibility) ──
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join(resource_paths),
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join(resource_paths),
        ),

        # ── Start Gazebo ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py',
                )
            ),
            launch_arguments={
                'gz_args': f'-r {os.path.join(pkg_share, "world", "swarm_world.sdf")}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # ── Dynamically spawn robots + bridge + teleop ──
        OpaqueFunction(function=_launch_setup),
    ])
