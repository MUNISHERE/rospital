from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rospital_gui',
            executable='rospital_gui_node',
            name='rospital_gui_node',
            output='screen'
        )
    ])