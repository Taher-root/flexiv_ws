from glob import glob

from setuptools import find_packages, setup

package_name = 'moveit_teleop_replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Aadith',
    maintainer_email='aadith.j@gmail.com',
    description='Replay recorded teleop episodes in RViz.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'episode_player = moveit_teleop_replay.episode_player:main',
            'tf_prefixer = moveit_teleop_replay.tf_prefixer:main',
            'description_publisher = moveit_teleop_replay.description_publisher:main',
        ],
    },
)
