from setuptools import setup

package_name = 'dss_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='me@example.com',
    description='DSS Controller with Python VSS Client',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'dss_controller = dss_controller.dss_controller_node:main',
            'imu_test_node = dss_controller.imu_test_node:main',
            'imu_integration_test = dss_controller.imu_integration_test:main',
            'imu_validation_node = dss_controller.imu_validation_node:main',
            'imu_vss_validation_node = dss_controller.imu_vss_validation_node:main',
            'vss_scanner_node = dss_controller.vss_scanner_node:main',
        ],
    },
)
