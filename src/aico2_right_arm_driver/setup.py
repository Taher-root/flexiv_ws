from setuptools import find_packages, setup

package_name = "aico2_right_arm_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            ["config/right_arm.yaml", "config/right_arm_hardware.yaml",
             "config/right_arm_hardware_moveit_servo.yaml",
             "config/right_arm_moveit_servo.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oelinux",
    maintainer_email="oelinux@todo.todo",
    description="Right Rizon arm driver for AICO2",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "right_arm_driver = aico2_right_arm_driver.right_arm_driver_node:main",
        ],
    },
)
