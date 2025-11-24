import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rospital_moveit_config_pkg = get_package_share_directory("rospital_moveit_config")

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Enable simulation mode'
    )
    with_octomap_arg = DeclareLaunchArgument(
        'with_octomap',
        default_value='false',
        description='Enable octomap generation'
    )
    is_sim = LaunchConfiguration('is_sim')
    with_octomap = LaunchConfiguration('with_octomap')

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rospital_description"),
            "launch",
            "gz.launch.py"
        ),
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
            "use_sim_time": "True"
        }.items()
    )

    moveit_controller = IncludeLaunchDescription(
            os.path.join(
                rospital_moveit_config_pkg,
                "launch",
                "moveit_controller.launch.py"
            ),
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                rospital_moveit_config_pkg,
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={
                "is_sim": is_sim,
                "with_octomap": with_octomap
            }.items()
        )


    return LaunchDescription([
        is_sim_arg,
        with_octomap_arg,
        gazebo,
        controller,
        joystick,
        moveit_controller,
        moveit
    ])