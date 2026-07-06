import os
from glob import glob
from setuptools import find_packages, setup

# 1. Den Paketnamen exakt so benennen wie in der package.xml und wie euer Ordner heißt
package_name = "rnm_needle"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Registriert alle Launch-Dateien im Ordner launch/
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        # NEU: Registriert eure YAML-Parameterdatei aus dem config/-Ordner im System
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    # 2. TODOs durch sinnvolle Werte ersetzen (passend zur package.xml)
    maintainer="Group X - RNM Project",
    maintainer_email="eure_gruppen_mail@tuhh.de",
    description="ROS 2 Python package for calculating the needle insertion path.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "insertion_path_node = rnm_needle.insertion_path_node:main",
        ],
    },
)
