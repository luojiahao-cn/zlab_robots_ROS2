import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory("zlab_robot_moveit_config")
    urdf_pkg_share = get_package_share_directory("zlab_robot_description")
    
    # Build MoveIt configuration for dual arm
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="zlab_robot_dual_arm",
            package_name="zlab_robot_moveit_config"
        )
        .robot_description(
            file_path=os.path.join(urdf_pkg_share, "urdf", "zlab_robot_dual_arm.urdf.xacro"),
            mappings={
                "config_file": os.path.join(urdf_pkg_share, "config", "robot_config_dual_arm.yaml"),
                "initial_positions_file": os.path.join(pkg_share, "config", "dual_arm", "initial_positions.yaml")
            }
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, "config", "dual_arm", "zlab_robot.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_share, "config", "dual_arm", "moveit_controllers.yaml")
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, "config", "dual_arm", "joint_limits.yaml")
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, "config", "kinematics.yaml")
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    
    # RViz configuration  
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("zlab_robot_moveit_config"), "rviz", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    
    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # ros2_control node
    ros2_controllers_path = os.path.join(pkg_share, "config", "dual_arm", "ros2_controllers.yaml")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Left arm controller spawner
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    
    # Right arm controller spawner
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )
    
    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )
    
    return LaunchDescription([
        rviz_node,
        static_tf_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        move_group_node,
    ])
