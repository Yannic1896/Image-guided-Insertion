"""Launch the needle insertion path planning node with its configuration."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # 1. getting the path to the configuration files
    kinematics_share = get_package_share_directory('rnm_kinematics') # alternativ mit:  # 1. Pfad zu DEINER eigenen needle_params.yaml holen
    needle_share = get_package_share_directory('rnm_needle')
    needle_params = os.path.join(needle_share, 'config', 'needle_params.yaml')
    needle_defaults = _load_needle_defaults(needle_params)
    panda_dh_params = os.path.join(kinematics_share, 'config', 'panda_dh.yaml')
    panda_limits = os.path.join(kinematics_share, 'config', 'panda_joint_limits.yaml')
    # 2. configuring the actual InsertionPathNode
    insertion_path_node = Node(
        package='rnm_needle',
        executable='insertion_path_node',
        name='insertion_path_node',
        output='screen',
        parameters=[
            panda_dh_params,
            panda_limits,
            needle_params,   # topics, DH-Parameters and limits
            {
                'input_source': LaunchConfiguration('input_source'),
                'target_location_file': LaunchConfiguration('target_location_file'),
                'entry_location_file': LaunchConfiguration('entry_location_file'),
                'entry_candidate_index': ParameterValue(
                    LaunchConfiguration('entry_candidate_index'),
                    value_type=int,
                ),
            },
        ], 
        
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('input_source', default_value='topic'),
            DeclareLaunchArgument(
                'target_location_file',
                default_value=needle_defaults.get(
                    'target_location_file',
                    '/workspaces/rnm-documents-primary/target_output/target_location.txt',
                ),
            ),
            DeclareLaunchArgument(
                'entry_location_file',
                default_value=needle_defaults.get(
                    'entry_location_file',
                    '/workspaces/rnm-documents-primary/entry_point_output/needle_entry_point.txt',
                ),
            ),
            DeclareLaunchArgument(
                'entry_candidate_index',
                default_value=str(needle_defaults.get('entry_candidate_index', 0)),
            ),
            insertion_path_node,
        ]
    )


def _load_needle_defaults(path: str) -> dict:
    with open(path, 'r') as file:
        data = yaml.safe_load(file) or {}
    return data.get('/**', {}).get('ros__parameters', {})
