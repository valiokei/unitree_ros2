#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def launch_setup(context, *args, **kwargs):
    # Robot description
    pkg_path = os.path.join(get_package_share_directory('a1_description'))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    
    # Parse xacro with use_gazebo=true to get fake_components and DEBUG=true for world frame
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_gazebo': 'true', 'DEBUG': 'true'})
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ROS2 Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory("unitree_controller"), "config", "a1_controllers.yaml")
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Unitree Controller
    unitree_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory('a1_description'), 'launch', 'a1.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Simple robot simulator for world movement
    simple_simulator = Node(
        package='unitree_teleop',
        executable='simple_robot_simulator',
        output='screen'
    )

    return [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        unitree_controller_spawner,
        rviz,
        simple_simulator
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
