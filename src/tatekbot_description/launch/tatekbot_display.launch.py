import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Set the path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('tatekbot_description'),
        'urdf',
        'tatekbotV2.urdf'
    )

    # Read the URDF file as a string
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Convert the URDF string to a parameter value
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=urdf_file,
        description='Absolute path to robot urdf file'
    ))

    # Robot state publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # Joint state publisher GUI
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    ))

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('tatekbot_description'),
        'rviz'
        'tatekbot.rviz'
    )
    


    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    ))

    return ld

