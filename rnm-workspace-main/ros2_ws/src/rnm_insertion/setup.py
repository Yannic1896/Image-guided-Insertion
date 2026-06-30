from glob import glob
from setuptools import find_packages, setup

package_name = "rnm_insertion"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(
        exclude=[
            "test",
            "rnm_sample",
            "rnm_sample.*",
            "rnm_needle",
            "rnm_needle.*",
        ],
    ),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                launch_file
                for launch_file in glob("launch/*.launch.py")
                if "rnm_sample" not in launch_file
            ],
        ),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Group X - RNM Project",
    maintainer_email="eure_gruppen_mail@tuhh.de",
    description="ROS 2 package for needle insertion planning and commands.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "insertion_node = rnm_insertion.insertion_node:main",
        ],
    },
)
