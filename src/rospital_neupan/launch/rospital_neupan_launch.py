from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rospital_neupan',
            executable='neupan_node',
            name='rospital_neupan_node',
            output='screen',
            parameters=['/path/to/rospital_neupan/config/neupan_params.yaml']
        )
    ])
