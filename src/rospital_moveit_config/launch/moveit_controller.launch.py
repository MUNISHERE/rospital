from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dual_arms_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arms_controller", 
            "--controller-manager",
            "/controller_manager",
        ],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[             
            "right_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    return LaunchDescription(
        [
            dual_arms_controller_spawner,
            left_gripper_controller_spawner,
            right_gripper_controller_spawner
        ]
    )