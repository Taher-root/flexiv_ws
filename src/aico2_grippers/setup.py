from setuptools import find_packages, setup

package_name = "aico2_grippers"

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
    maintainer="oelinux",
    maintainer_email="oelinux@todo.todo",
    description="Flexiv RDK gripper helpers and force-grasp teleop logic",
    license="Proprietary",
    tests_require=["pytest"],
)
