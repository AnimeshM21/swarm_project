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
        distance = sqrt((new_pos[0] - pos[0])**2 + (new_pos[1] - pos[1])**2)
        if distance < min_distance:
            return False
    return True

def generate_launch_description():
    # 1. Setup Paths 
    package_name = 'swarm'
    pkg_share = get_package_share_directory(package_name)
    
    world_path = os.path.join(pkg_share, 'world')
    model_path = os.path.join(pkg_share, 'model')
    install_dir = os.path.dirname(pkg_share)
    
    world_file = os.path.join(world_path, 'swarm_world.sdf')
    xacro_file = os.path.join(model_path, 'robot.xacro')
    bridge_params = os.path.join(pkg_share, 'parameters', 'bridge_params.yaml')

    # 2. Environment Variables
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([world_path, install_dir])
    )
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([world_path, install_dir])
    )

    # 3. Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # 4. Generate Positions
    num_bots = 3 #have to make relevant changes in swarm_teleop as well and bridge_params
    min_distance = 1.0
    initial_conditions = []
    np.random.seed(21)# spawning in same location everytime
    
    for _ in range(num_bots):
        attempts = 0
        while attempts < 100:
            x = np.random.uniform(-2.0, 2.0)
            y = np.random.uniform(-2.0, 2.0)
            if is_valid_position([x, y], initial_conditions, min_distance):
                initial_conditions.append([x, y, 0.2, 0.0]) # x, y, z, yaw
                break
            attempts += 1

    # 5. Build Launch Description
    ld = LaunchDescription()
    ld.add_action(set_ign_resource_path)
    ld.add_action(set_gz_resource_path)
    ld.add_action(gazebo_launch)

    # 6. Main Loop logic for spwaning multiple bots without conflict

    for i, pos in enumerate(initial_conditions):
        robot_name = f"ground_bot_{i+1}"
        ns = f"bot_{i+1}"

        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, ' frame_prefix:=', f'{ns}/']),
            value_type=str
        )

        robot_group = GroupAction([
            PushRosNamespace(ns),
            
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
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
                    '-z', str(pos[2]),
                    '-Y', str(pos[3])
                ],
                output='screen'
            ),

            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_params, 'use_sim_time': True}]
            )
        ])
        ld.add_action(robot_group)

    return ld