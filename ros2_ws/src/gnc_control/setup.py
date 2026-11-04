from glob import glob

from setuptools import setup

package_name = 'gnc_control'

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
    description='Navigation and control nodes of the robot fish.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_control = gnc_control.heading_control_node:main',   # heading controller
            'heading_test = gnc_control.heading_test_node:main',         # scripted heading setpoints + CSV log
            'tail_oscillator = gnc_control.tail_oscillator_node:main',   # real robot: flapping → tail servo angle
            'fusion = gnc_control.fusion_node:main',                     # orientation from the raw sensors
            'navigation = gnc_control.navigation_node:main',             # heading, depth and vertical speed
        ],
    },
)
