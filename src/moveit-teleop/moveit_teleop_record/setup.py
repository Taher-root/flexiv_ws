from setuptools import find_packages, setup

package_name = 'moveit_teleop_record'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        # Run in OTHER environments, never imported by ROS code.
        ('share/' + package_name + '/training',
         ['training/verify_gr00t.py']),
        ('share/' + package_name + '/scripts',
         ['scripts/setup_lerobot_env.sh', 'scripts/setup_gr00t_check_env.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aadith',
    maintainer_email='aadith.j@gmail.com',
    description='Episode recording for MoveIt teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'episode_recorder = moveit_teleop_record.episode_recorder:main',
            'convert = moveit_teleop_record.convert:main',
            'view = moveit_teleop_record.convert:view',
        ],
    },
)
