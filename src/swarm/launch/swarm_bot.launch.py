import os
import numpy as np
from math import sqrt
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue

def is_valid_position(new_pos, existing_positions, min_distance):
    for pos in existing_positions:
        if sqrt((new_pos[0]-pos[0])**2 + (new_pos[1]-pos[1])**2) < min_distance: return False
    return True

def generate_launch_description():
    pkg_share = get_package_share_directory('swarm')
    world_file = os.path.join(pkg_share, 'world', 'swarm_world.sdf')
    xacro_file = os.path.join(pkg_share, 'model', 'robot.xacro')
    bridge_params = os.path.join(pkg_share, 'parameters', 'bridge_params.yaml')

    # Path setup to fix "Unable to find uri" errors
    # We include both the 'model' directory and the 'share' directory itself
    resource_paths = [
        os.path.join(pkg_share, 'model'),
        os.path.join(pkg_share, 'world'),
        os.path.dirname(pkg_share) 
    ]

    set_gz_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths)
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    ld = LaunchDescription([set_gz_path, gazebo_launch])

    num_bots = 3
    initial_conditions = []
    np.random.seed(21)
    for _ in range(num_bots):
        while True:
            pos = [np.random.uniform(-2, 2), np.random.uniform(-2, 2)]
            if is_valid_position(pos, initial_conditions, 1.2):
                initial_conditions.append([pos[0], pos[1], 0.1, 0.0])
                break

    for i, pos in enumerate(initial_conditions):
        ns = f"bot_{i+1}"
        robot_name = f"ground_bot_{i+1}"

        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, ' frame_prefix:=', f'{ns}/']),
            value_type=str
        )

        ld.add_action(GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot_name, 
                    '-topic', 'robot_description', 
                    '-x', str(pos[0]), 
                    '-y', str(pos[1]), 
                    '-z', str(pos[2])
                ]
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_params, 'use_sim_time': True}]
            )
        ]))

    return ld