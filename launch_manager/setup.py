from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'launch_manager'

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
        (os.path.join('share', package_name, 'ui'), glob('launch_manager/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='amap@todo.todo',
    description='ROS2 Launch Manager with Qt GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_manager = launch_manager.launch_manager_node:main',
        ],
    },
)
