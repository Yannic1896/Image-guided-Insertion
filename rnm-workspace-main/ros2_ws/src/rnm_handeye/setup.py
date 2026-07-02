from glob import glob

from setuptools import find_packages, setup

package_name = "rnm_handeye"

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
        ("share/" + package_name + "/config",
         glob("config/*"),)
    ],
    install_requires=["setuptools","numpy","pyyaml"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@todo.todo",
    description="Hand-eye calibration package for Panda and Azure Kinect.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chessboard_node = rnm_handeye.chessboard_node:main",
            "collector_node = rnm_handeye.collector_node:main",
            "calc_handeye = rnm_handeye.calc_handeye:main",
        ],
    },
)
