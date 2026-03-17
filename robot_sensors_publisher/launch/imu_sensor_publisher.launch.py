from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default='')

    imu_node = Node(
        package='robot_sensors_publisher',
        executable='imu_serial_publisher',
        name='imu_serial_publisher',
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
        DeclareLaunchArgument('use_rviz', default_value='false', description='Ativa RViz (true/false)'),
        DeclareLaunchArgument('rviz_config', default_value='', description='Caminho para o arquivo de configuração do RViz'),
        imu_node,
        rviz_node
    ])