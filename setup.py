from setuptools import setup
import os
from glob import glob

package_name = 'px4_mpc_offboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='MPC-based offboard control for PX4 via MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_waypoint_follower_mavros = px4_mpc_offboard.mpc_waypoint_follower_mavros:main',
            'waypoint_publisher = px4_mpc_offboard.waypoint_publisher:main',
            'data_logger = px4_mpc_offboard.data_logger:main',
        ],
    },
)
