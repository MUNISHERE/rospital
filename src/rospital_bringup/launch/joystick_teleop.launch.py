import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Lấy đường dẫn tới gói 'rospital_bringup'
    rospital_pkg = get_package_share_directory('rospital_bringup')

    # Tạo biến cấu hình 'use_sim_time'
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    # Node điều khiển từ joystick (joy_teleop)
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(rospital_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Node xử lý tín hiệu từ joystick (joy_node)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(rospital_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ]
    )
    # Bao gồm file launch của twist_mux
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
        ),
        launch_arguments={
            "cmd_vel_out": "rospital_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(rospital_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(rospital_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(rospital_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Node chuyển đổi tín hiệu Twist
    twist_relay_node = Node(
        package="rospital_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Trả về LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
        ]
    )
