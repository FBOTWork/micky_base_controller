#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default='')

    imu_serial_publisher_node = Node(
        package='robot_sensors_publisher',
        executable='imu_serial_publisher',
        name='imu_serial_publisher_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baud': 115200},
            {'frame_id': 'imu_link'}
        ]
    )

    inverse_kinematic_node = Node(
        package='motors_controller',
        executable='inverse_kinematic',
        name='inverse_kinematic_node',
        output='screen',
        parameters=[
            {'motor_port': '/dev/ttyUSB1'},
            {'motor_baud': 115200}
        ]

    )

    cmd_vel_to_odom_node = Node(
        package='motors_controller',
        executable='cmd_vel_to_odom',
        name='cmd_vel_to_odom_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false', description='Activate RViz'),
        DeclareLaunchArgument('rviz_config', default_value='', description='Path to the RViz configuration file'),
        imu_serial_publisher_node,
        inverse_kinematic_node,
        cmd_vel_to_odom_node,
        rviz_node
    ])