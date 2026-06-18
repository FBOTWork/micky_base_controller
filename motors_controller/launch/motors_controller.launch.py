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

    cmd_vel_node = Node(
        package='motors_controller',
        executable='cmd_vel_to_serial',
        name='cmd_vel_to_serial_ros2',
        output='screen',
        parameters=[
            {'port': '/dev/arduino_robo'},
            {'baudrate': 115200}
        ]

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
        cmd_vel_node,
        rviz_node
    ])
