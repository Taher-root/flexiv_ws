from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flexiv_amr_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oelinux',
    maintainer_email='oelinux@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_controller = flexiv_amr_driver.velocity_controller:main',
            'robokit_velocity_controller = flexiv_amr_driver.robokit_velocity_controller:main',
            'odometry_publisher = flexiv_amr_driver.odometry_publisher:main',
            'seer_lidar_publisher = flexiv_amr_driver.seer_lidar_publisher:main',
            'seer_imu_publisher = flexiv_amr_driver.seer_imu_publisher:main',
            'status_monitor = flexiv_amr_driver.status_monitor:main',
            'joint_state_merger = flexiv_amr_driver.joint_state_merger:main',
        ],
    },
)
