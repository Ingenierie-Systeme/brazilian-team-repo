from setuptools import find_packages, setup

package_name = 'auv_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniefss20@gmail.com',
    description='ROS 2 hardware driver nodes for Blue Robotics Navigator, Bar30, Servos, and Camera',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bar30_publisher = auv_hardware.bar30_publisher:main',
            'servo_subscriber = auv_hardware.servo_subscriber:main',
            'camera_publisher = auv_hardware.camera_publisher:main',
            'imu_publisher = auv_hardware.imu_publisher:main',
            'teleop_controller = auv_hardware.teleop_controller:main',
        ],
    },
)
