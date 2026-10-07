from setuptools import setup
from glob import glob

package_name = 'aico2_vr_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/webxr', glob('webxr/*.html')),
        ('share/' + package_name + '/webxr/static', glob('webxr/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vr_bridge = aico2_vr_teleop.vr_bridge_node:main',
        ],
    },
)
