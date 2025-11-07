from setuptools import setup
import os
from glob import glob

package_name = 'slam_frontend'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='todo@umd.edu',
    description='SLAM frontend implementation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_estimator_node = slam_frontend.odometry_estimator_node:main',
            'local_map_manager_node = slam_frontend.local_map_manager_node:main',
            'minimal_test_node = slam_frontend.minimal_test:main',
        ],
    },
)
