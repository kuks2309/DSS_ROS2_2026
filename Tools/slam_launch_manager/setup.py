from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'slam_launch_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['ui/*.ui'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('slam_launch_manager/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='amap@todo.todo',
    description='ROS2 SLAM Launch Manager with Qt GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_launch_manager = slam_launch_manager.slam_launch_manager_node:main',
        ],
    },
)
