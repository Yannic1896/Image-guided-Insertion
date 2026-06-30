from glob import glob

from setuptools import find_packages, setup

package_name = "rnm_tools"


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
        (
            "share/" + package_name + "/config/sim",
            ["config/sim/single_sim_controllers.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lshala",
    maintainer_email="lshala@todo.todo",
    description="Utilities for the RNM ROS 2 workspace.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "urdf_visualizer = rnm_tools.urdf_visualizer:main",
            "joint_command_publisher = rnm_tools.joint_command_publisher:main",
            "camera_static_tf_publisher = rnm_tools.camera_static_tf_publisher:main",
        ],
    },
)
