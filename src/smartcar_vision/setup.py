from glob import glob
from setuptools import find_packages, setup


package_name = "smartcar_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="LX-HMKK",
    maintainer_email="lx_hmkk@qq.com",
    description="QR recognition and bounded local VLM services for SmartCar.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_node = smartcar_vision.vision_node:main",
        ],
    },
)
