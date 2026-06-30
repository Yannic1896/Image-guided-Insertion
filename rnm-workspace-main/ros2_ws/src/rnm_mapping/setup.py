from glob import glob

from setuptools import find_packages, setup

package_name = "rnm_mapping"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lshala",
    maintainer_email="lshala@todo.todo",
    description="Point cloud accumulation and mapping tools for the RNM workspace.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cloud_stitcher = rnm_mapping.cloud_stitcher:main",
            "register_stl_to_scan = rnm_mapping.model_registration:main",
            "locate_stl_target = rnm_mapping.target_locator:main",
            "find_needle_entry = rnm_mapping.needle_entry_planner:main",
            (
                "publish_target_entry_pose_array = "
                "rnm_mapping.target_entry_pose_array_publisher:main"
            ),
        ],
    },
)
