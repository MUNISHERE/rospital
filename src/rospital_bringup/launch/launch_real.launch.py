import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rospital_bringup"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("rospital_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rospital_bringup"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rospital_bringup"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        laser_driver,
        joystick
    ])