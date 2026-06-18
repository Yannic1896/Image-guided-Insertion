from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Chessboard Detection Node
        Node(
            package='rnm_handeye',
            executable='chessboard_node',
            name='chessboard_node',
            output='screen'
        ),

        # 2. Start the Data Collector Node 
        Node(
            package='rnm_handeye',
            executable='collector_node',
            name='collector_node',
            output='screen',
            prefix="xterm -e", 
            emulate_tty=True
        ),
        
    ])