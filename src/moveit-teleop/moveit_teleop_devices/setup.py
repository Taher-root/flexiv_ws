from glob import glob

from setuptools import find_packages, setup

package_name = 'moveit_teleop_devices'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        # The WebXR page the headset loads, and the cert helper.
        ('share/' + package_name + '/web', glob('web/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aadith',
    maintainer_email='aadith.j@gmail.com',
    description='Teleoperation input device drivers for MoveIt teleoperation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_device = moveit_teleop_devices.fake_device:main',
            'xr_device = moveit_teleop_devices.xr_device:main',
        ],
        # The device plugin seam. Third parties add drivers by declaring this
        # entry point group from their own package -- no edits here required.
        'moveit_teleop.devices': [
            'fake = moveit_teleop_devices.fake_device:FakeDevice',
            'xr = moveit_teleop_devices.xr_device:XRDevice',
        ],
    },
)
