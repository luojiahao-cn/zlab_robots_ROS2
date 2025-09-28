#!/usr/bin/env python3


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 获取包路径
    pkg_share = FindPackageShare(package="zlab_robot_description")

    # 获取URDF文件路径
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "zlab_robot.urdf.xacro"])

    # 获取RViz配置文件路径
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "view_robot.rviz"])

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="是否使用仿真时间"
    )

    arm_id_arg = DeclareLaunchArgument(
        "arm_id", default_value="zlab_arm", description="机械臂命名空间/前缀"
    )

    parent_link_arg = DeclareLaunchArgument(
        "parent_link", default_value="world", description="机械臂连接到的父链接"
    )

    base_xyz_arg = DeclareLaunchArgument(
        "base_xyz", default_value="0 0 0", description="基座在父链接坐标系下的位置"
    )

    base_rpy_arg = DeclareLaunchArgument(
        "base_rpy", default_value="0 0 0", description="基座在父链接坐标系下的RPY姿态"
    )

    joint_state_gui_arg = DeclareLaunchArgument(
        "joint_state_gui", default_value="true", description="是否启用关节状态GUI"
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " ",
            "arm_id:=",
            LaunchConfiguration("arm_id"),
            " ",
            "parent_link:=",
            LaunchConfiguration("parent_link"),
            " ",
            'base_xyz:="',
            LaunchConfiguration("base_xyz"),
            '" ',
            'base_rpy:="',
            LaunchConfiguration("base_rpy"),
            '"',
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_description_content},
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=UnlessCondition(LaunchConfiguration("joint_state_gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("joint_state_gui")),
    )

    # RViz - 可视化工具
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            arm_id_arg,
            parent_link_arg,
            base_xyz_arg,
            base_rpy_arg,
            joint_state_gui_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
