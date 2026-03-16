from setuptools import setup
import os
import glob

package_name = 'motors_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fbot',
    description='Motor controller node',
    license='Apache 2.0',

    entry_points={
        'console_scripts': [
        'cmd_vel_to_serial = motors_controller.cmd_vel_to_serial_ros2:main'        
        ],
    },
)