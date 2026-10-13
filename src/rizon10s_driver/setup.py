from setuptools import find_packages, setup

package_name = "rizon10s_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/arm_driver.launch.py"]),
        (
            f"share/{package_name}/config",
            ["config/arm.yaml", "config/arm_hardware.yaml",
             "config/arm_hardware_moveit_servo.yaml",
             "config/arm_moveit_servo.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taher Kapadia",
    maintainer_email="taherk@seas.upenn.edu",
    description="Rizon 10s single-arm driver (Flexiv RDK) with lifecycle and mock mode",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_driver = rizon10s_driver.arm_driver_entry:main",
        ],
    },
)
