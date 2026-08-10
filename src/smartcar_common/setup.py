from setuptools import find_packages, setup


package_name = "smartcar_common"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartCar Team",
    maintainer_email="maintainers@example.com",
    description="Small runtime contracts shared by SmartCar ROS packages.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
