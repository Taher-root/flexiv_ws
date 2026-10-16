from setuptools import find_packages, setup

package_name = 'moveit_teleop_core'

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
    description='Robot-side teleoperation logic for MoveIt.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'retarget_node = moveit_teleop_core.retarget_node:main',
            'servo_session = moveit_teleop_core.servo_session_node:main',
            'mode_manager = moveit_teleop_core.mode_manager:main',
            'gripper_node = moveit_teleop_core.gripper_node:main',
            'joint_limit_guard = moveit_teleop_core.joint_limit_guard:main',
            'scene_objects = moveit_teleop_core.scene_objects:main',
        ],
        # The retargeting plugin seam. A bimanual setup or a dexterous hand
        # adds a strategy by declaring this group from its own package; no
        # edits here are required.
        'moveit_teleop.retargeters': [
            'relative_pose = moveit_teleop_core.retargeting.relative_pose:RelativePoseRetargeter',
        ],
    },
)
