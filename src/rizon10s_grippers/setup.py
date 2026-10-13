from setuptools import find_packages, setup

package_name = "rizon10s_grippers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/gripper_defaults.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Taher Kapadia",
    maintainer_email="taherk@seas.upenn.edu",
    description="Flexiv RDK gripper helpers for Rizon 10s VR teleop",
    license="Proprietary",
    tests_require=["pytest"],
)
