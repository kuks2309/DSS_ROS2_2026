from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bag_replay_tool_ros2'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='amap@todo.todo',
    description='GUI tool for replaying ROS2 bag files with topic selection and playback control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_replay_tool_node = bag_replay_tool_ros2.bag_replay_tool_node:main',
        ],
    },
)
