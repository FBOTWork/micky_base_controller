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

    inverse_kinematic_node = Node(
        package='motors_controller',
        executable='inverse_kinematic',
        name='inverse_kinematic_node',
        output='screen',
        parameters=[
            {'motor_port': '/dev/base_controller'},
            {'motor_baud': 115200},
            {'imu_frame_id': 'imu_link'}
        ]

    )

    imu_odom_node = Node(
        package='motors_controller',
        executable='imu_odom_node',
        name='imu_odom_node',
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
        inverse_kinematic_node,
        imu_odom_node,
        rviz_node
    ])