from setuptools import find_packages, setup

package_name = 'moveit_teleop_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aadith',
    maintainer_email='aadith.j@gmail.com',
    description='Full-IK teleoperation solver for MoveIt manipulators.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = moveit_teleop_ik.ik_node:main',
        ],
    },
)
