from glob import glob

from setuptools import find_packages, setup

package_name = "rnm_scanning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    description="Scanning node to sync images with robot motion.",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scanning_node = rnm_scanning.scanning_node:main",
        ],
    },
)
