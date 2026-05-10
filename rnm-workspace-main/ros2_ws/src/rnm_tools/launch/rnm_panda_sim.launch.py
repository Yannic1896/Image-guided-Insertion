from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def concatenate_ns(ns1, ns2, absolute=False):
    if len(ns1) == 0:
        return ns2
    if len(ns2) == 0:
        return ns1

    if ns1[0] == "/":
        ns1 = ns1[1:]
    if ns1[-1] == "/":
        ns1 = ns1[:-1]
    if ns2[0] == "/":
        ns2 = ns2[1:]
    if ns2[-1] == "/":
        ns2 = ns2[:-1]
    if absolute:
        ns1 = "/" + ns1
    return ns1 + "/" + ns2


def generate_launch_description():
    arm_id_param = "arm_id"
    initial_positions_param = "initial_positions"
    viser_host_param = "viser_host"
    viser_port_param = "viser_port"
    unpause_param = "unpause"
    realtime_param = "realtime"

    arm_id = LaunchConfiguration(arm_id_param)
    initial_positions = LaunchConfiguration(initial_positions_param)
    viser_host = LaunchConfiguration(viser_host_param)
    viser_port = LaunchConfiguration(viser_port_param)
    unpause = LaunchConfiguration(unpause_param)
    realtime = LaunchConfiguration(realtime_param)

    load_gripper = True

    if load_gripper:
        scene_file = "scene.xml"
    else:
        scene_file = "scene_ng.xml"

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "sim",
        "panda_arm_sim.urdf.xacro",
    )
    xml_file = os.path.join(
        get_package_share_directory("franka_description"),
        "mujoco",
        "franka",
        scene_file,
    )
    rnm_tools_path = get_package_share_directory("rnm_tools")
    mjros_config_file = os.path.join(
        rnm_tools_path,
        "config",
        "sim",
        "single_sim_controllers.yaml",
    )
    ns = ""
    controller_manager_name = concatenate_ns(ns, "controller_manager", True)

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " arm_id:=",
            arm_id,
            " hand:=",
            str(load_gripper).lower(),
            " initial_positions:=",
            initial_positions,
        ]
    )

    params = {"robot_description": robot_description}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=ns,
        parameters=[params],
    )

    jsp_source_list = [concatenate_ns(ns, "joint_states", True)]
    if load_gripper:
        jsp_source_list.append(concatenate_ns(ns, "panda_gripper_sim_node/joint_states", True))

    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=ns,
        parameters=[{"source_list": jsp_source_list, "rate": 30}],
    )

    node_mujoco = Node(
        package="mujoco_ros",
        executable="mujoco_node",
        output="screen",
        arguments=[
            "--admin-hash",
            "''",
            "--ros-args",
            "--log-level",
            "mujoco_server:=info",
            "--log-level",
            "mujoco_ros_plugin_loader:=info",
            "--log-level",
            "Viewer:=warn",
        ],
        parameters=[
            {
                "use_sim_time": True,
                "ns": ns,
                "modelfile": xml_file,
                "unpause": unpause,
                "headless": True,
                "render_offscreen": False,
                "no_render": True,
                "wait_for_xml": False,
                "realtime": realtime,
                "eval_mode": False,
                "num_steps": -1,
                "num_mj_threads": 1,
            },
            mjros_config_file,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
        output="screen",
    )

    franka_robot_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_state_broadcaster", "-c", controller_manager_name],
        output="screen",
    )

    franka_robot_model_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_model_broadcaster", "-c", controller_manager_name],
        output="screen",
    )

    joint_position_example_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_example_controller", "-c", controller_manager_name],
        output="screen",
    )

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rnm_tools_path, "launch", "visualizer.launch.py")
        ),
        launch_arguments={
            "viser_host": viser_host,
            "viser_port": viser_port,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                arm_id_param,
                default_value="panda",
                description="The name of the robot. Defaults to panda.",
            ),
            DeclareLaunchArgument(
                initial_positions_param,
                default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
                description=(
                    "Initial joint positions of the robot. Must be enclosed in quotes, "
                    'and in pure number. Defaults to the "communication_test" pose.'
                ),
            ),
            DeclareLaunchArgument(
                viser_host_param,
                default_value="0.0.0.0",
                description="Host address for the RNM web visualizer.",
            ),
            DeclareLaunchArgument(
                viser_port_param,
                default_value="8080",
                description="Port for the RNM web visualizer.",
            ),
            DeclareLaunchArgument(
                unpause_param,
                default_value="true",
                description="Whether to start the MuJoCo simulation unpaused.",
            ),
            DeclareLaunchArgument(
                realtime_param,
                default_value="1.0",
                description="Fraction of realtime for the MuJoCo simulation. Lower values reduce CPU use.",
            ),
            node_robot_state_publisher,
            node_joint_state_publisher,
            node_mujoco,
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[franka_robot_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=franka_robot_state_broadcaster_spawner,
                    on_exit=[franka_robot_model_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=franka_robot_model_broadcaster_spawner,
                    on_exit=[joint_position_example_controller_spawner],
                )
            ),
            visualizer_launch,
        ]
    )
