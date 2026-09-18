from setuptools import find_packages, setup

package_name = "aico2_waist_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/waist_driver.launch.py"]),
        (f"share/{package_name}/config", ["config/waist_driver.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oelinux",
    maintainer_email="oelinux@todo.todo",
    description=(
        "Waist (AGV_Joint1/2) driver: publishes the shared external axes "
        "from one Rizon controller's RDK stream as their own JointState "
        "source, independent of either arm driver."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waist_driver = aico2_waist_driver.waist_driver_node:main",
        ],
    },
)
