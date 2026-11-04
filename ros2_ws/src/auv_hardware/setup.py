from glob import glob

from setuptools import setup

package_name = 'auv_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),   # makes the package findable
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),                  # parameters
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),             # launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedrohaas',
    maintainer_email='pedrohaa.silva@gmail.com',
    description='Drivers of the real robot fish: Navigator board and pressure sensor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = auv_hardware.navigator_node:main',   # IMU, magnetometer and servos of the Navigator board
            'barometer = auv_hardware.barometer_node:main',   # pressure sensor
        ],
    },
)
