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
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_parent_frame = LaunchConfiguration('imu_parent_frame')
    imu_front_x = LaunchConfiguration('imu_front_x')
    imu_rear_x = LaunchConfiguration('imu_rear_x')

    inverse_kinematic_node = Node(
        package='motors_controller',
        executable='inverse_kinematic',
        name='inverse_kinematic_node',
        output='screen',
        parameters=[
            {'motor_port': '/dev/stm_controller'},
            {'motor_baud': 115200},
            {'imu_frame_id': 'imu_link'},
            {'imu_front_frame_id': 'imu_front_link'},
            {'imu_rear_frame_id': 'imu_rear_link'}
        ]

    )

    imu_front_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_front_tf',
        arguments=['--x', imu_front_x, '--frame-id', imu_parent_frame, '--child-frame-id', 'imu_front_link'],
        condition=IfCondition(publish_imu_tf)
    )

    imu_rear_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_rear_tf',
        arguments=['--x', imu_rear_x, '--frame-id', imu_parent_frame, '--child-frame-id', 'imu_rear_link'],
        condition=IfCondition(publish_imu_tf)
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
        DeclareLaunchArgument('publish_imu_tf', default_value='true', description='Publish static TFs for the front and rear IMUs'),
        DeclareLaunchArgument('imu_parent_frame', default_value='base_link', description='Parent frame of the IMU TFs'),
        DeclareLaunchArgument('imu_front_x', default_value='0.245', description='X offset (m) of the front IMU'),
        DeclareLaunchArgument('imu_rear_x', default_value='-0.325', description='X offset (m) of the rear IMU'),
        inverse_kinematic_node,
        imu_front_tf_node,
        imu_rear_tf_node,
        imu_odom_node,
        rviz_node
    ])