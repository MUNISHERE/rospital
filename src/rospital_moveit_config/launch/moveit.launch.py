#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    is_sim = LaunchConfiguration('is_sim')
    with_octomap = LaunchConfiguration('with_octomap')

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )
    with_octomap_arg = DeclareLaunchArgument(
        'with_octomap',
        default_value='false'
    )
    initial_positions_file_path = os.path.join(
        get_package_share_directory('rospital_moveit_config'),
        'config',
        'initial_positions.yaml'
    )
    moveit_config = (
        MoveItConfigsBuilder("rospital", package_name="rospital_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("rospital_description"),
            "urdf",
            "rospital.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/rospital.srdf")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .sensors_3d(file_path="config/sensor_3d.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    move_group_params = moveit_config.to_dict()
    move_group_params.update({
        'use_sim_time': is_sim,
        'start_state': {'content': initial_positions_file_path},
        'publish_robot_description_semantic': True,
    })

    move_group_params_no_sensors = dict(move_group_params)
    move_group_params_no_sensors.pop("sensors", None)

    move_group_with_octomap_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params], 
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(with_octomap) 
    )

    move_group_no_octomap_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params_no_sensors], 
        arguments=["--ros-args", "--log-level", "info"],
        condition=UnlessCondition(with_octomap) 
    )

        # RViz
    #rviz_config = os.path.join(
    #    get_package_share_directory("ur5_bringup"),
    #        "config",
    #        "moveit.rviz",
    #)
    #rviz_node = Node(
        #package="rviz2",
        #executable="rviz2",
        #name="rviz2",
        #output="log",
        #arguments=["-d", rviz_config],
        #parameters=[
            #moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            #moveit_config.robot_description_kinematics,
            #moveit_config.joint_limits,
        #],
    #)

    return LaunchDescription(
        [
            is_sim_arg,
            with_octomap_arg, 
            move_group_with_octomap_node, 
            move_group_no_octomap_node 
            #rviz_node
        ]
    )