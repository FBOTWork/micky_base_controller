from setuptools import find_packages, setup

package_name = 'robot_sensors_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/robot_sensors_publisher/launch', ['launch/imu_sensor_publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fbot',
    maintainer_email='kauasilvaolivera998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	'imu_serial_publisher = robot_sensors_publisher.imu_serial_publisher:main',
        ],
    },
)
