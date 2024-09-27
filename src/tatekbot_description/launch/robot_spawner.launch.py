#!/usr/bin/env python3
from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_name = LaunchConfiguration("tatekbot")

   
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    launch_file_dir = os.path.join(get_package_share_directory('tatekbot_description'), 'launch')
    pkg_four_ws_control = get_package_share_directory('tatekbot_control')
    
    
    
    
    

    start_spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name,
            "-x",
            "9.4102959851406",
            "-y",
            "-9.249168178507047",
            "-z",
            "0.35",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            "1.5853214457150695",
        ],
    )


    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
            ),
        )

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
        
    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )

    nodes = [
         
        robot_state_publisher,
        joy_node,
        controller,
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster
    ]
		

    ld = LaunchDescription(nodes)
    
    ld.add_action(start_spawn_entity_cmd)

    return ld
