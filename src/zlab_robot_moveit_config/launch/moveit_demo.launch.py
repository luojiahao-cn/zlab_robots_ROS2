#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """参数化的 MoveIt demo launch 文件，支持单臂/双臂切换"""

    # 获取包路径
    moveit_config_pkg = get_package_share_directory("zlab_robot_moveit_config")
    robot_description_pkg = get_package_share_directory("zlab_robot_description")

    # 声明启动参数
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation time"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_config",
            default_value="dual",
            description="Arm configuration: single_left, single_right, or dual",
            choices=["single_left", "single_right", "dual"],
        )
    )

    # 获取启动配置
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_config = LaunchConfiguration("arm_config")

    # 根据 arm_config 设置配置文件路径
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("zlab_robot_description"),
            "config",
            "zlab_robots_config.yaml",
        ]
    )

    # Robot description with xacro processing
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("zlab_robot_description"),
                    "urdf",
                    "zlab_robot.urdf.xacro",
                ]
            ),
            " config_file:=",
            config_file,
            " arm_config:=",
            arm_config,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot semantic description
    srdf_file = os.path.join(moveit_config_pkg, "config", "zlab_robot.srdf")
    robot_description_semantic_content = open(srdf_file).read()
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics configuration
    kinematics_file = os.path.join(moveit_config_pkg, "config", "kinematics.yaml")
    kinematics_yaml = open(kinematics_file).read()
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # Planning scene monitor configuration
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # 定义节点
    nodes = []

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    nodes.append(robot_state_publisher)

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(joint_state_publisher_gui)

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )
    nodes.append(move_group_node)

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
    )
    nodes.append(rviz_node)

    return LaunchDescription(declared_arguments + nodes)


if __name__ == "__main__":
    generate_launch_description()
