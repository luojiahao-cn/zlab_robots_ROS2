#!/usr/bin/env python3
"""
zlab_robot_description RViz 启动文件

用于在 RViz 中可视化机械臂模型：
- 从配置文件加载机械臂配置
- 支持多机械臂配置
- 自动生成关节状态发布器
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Any, List, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


class ZlabRobotRvizLauncher:
    """zlab机械臂RViz启动器"""

    def __init__(self):
        self.pkg_share = get_package_share_directory("zlab_robot_description")
        self.urdf_file = Path(self.pkg_share) / "urdf" / "zlab_robot.urdf.xacro"
        self.rviz_config_file = Path(self.pkg_share) / "config" / "view_robot.rviz"
        self.default_config_file = (
            Path(self.pkg_share) / "config" / "zlab_robots_config.yaml"
        )

    def get_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """定义启动参数"""
        return [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时间",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(self.default_config_file),
                description="机械臂配置文件（YAML）路径",
            ),
            DeclareLaunchArgument(
                "joint_state_gui",
                default_value="true",
                description="是否启用关节状态GUI",
                choices=["true", "false"],
            ),
        ]

    def load_robot_config(self, config_path: str) -> Tuple[Dict[str, Any], bool]:
        """
        加载机器人配置文件

        Returns:
            (config_data, is_valid): 配置数据和有效性标志
        """
        resolved_path = Path(config_path).expanduser().resolve()

        if not resolved_path.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_path}")

        try:
            with open(resolved_path, "r", encoding="utf-8") as f:
                config_data = yaml.safe_load(f) or {}
            print(f"成功加载配置文件: {resolved_path}")
            return config_data, True
        except Exception as e:
            raise Exception(f"加载配置文件失败 '{resolved_path}': {e}")

    def parse_arms_config(
        self, config_data: Dict[str, Any]
    ) -> Tuple[List[str], List[str]]:
        """
        解析机械臂配置

        Returns:
            (enabled_arms, joint_names): 启用的机械臂列表和关节名称列表
        """
        enabled_arms = []
        joint_names = []

        arms_data = config_data.get("arms", {})
        if not isinstance(arms_data, dict):
            return enabled_arms, joint_names

        for arm_name, arm_cfg in arms_data.items():
            if not isinstance(arm_cfg, dict) or not arm_cfg.get("enabled", False):
                continue

            # 处理前缀
            raw_prefix = arm_cfg.get("prefix", arm_name)
            prefix = raw_prefix if raw_prefix.endswith("_") else f"{raw_prefix}_"

            # 生成关节名称
            dof = int(arm_cfg.get("dof", 6))
            arm_joint_names = [f"{prefix}j{i+1}" for i in range(dof)]

            joint_names.extend(arm_joint_names)
            enabled_arms.append(f"{arm_name}({prefix.rstrip('_')})")

        return enabled_arms, joint_names

    def create_robot_description(self, context, config_file: str) -> Command:
        """创建机器人描述命令"""
        return Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(self.urdf_file),
                " ",
                f"config_file:={config_file}",
            ]
        )

    def create_nodes(
        self, context, joint_names: List[str], config_file: str
    ) -> List[Node]:
        """创建所有节点"""
        use_sim_time = LaunchConfiguration("use_sim_time")

        # Robot State Publisher
        robot_description = self.create_robot_description(context, config_file)
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_description": robot_description},
            ],
        )

        # Joint State Publisher (without GUI)
        joint_state_params = [{"use_sim_time": use_sim_time}]
        if joint_names:
            joint_state_params.append({"joint_names": joint_names})

        joint_state_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=joint_state_params,
            condition=UnlessCondition(LaunchConfiguration("joint_state_gui")),
        )

        # Joint State Publisher GUI
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=joint_state_params,
            condition=IfCondition(LaunchConfiguration("joint_state_gui")),
        )

        # RViz2
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(self.rviz_config_file)],
            parameters=[{"use_sim_time": use_sim_time}],
        )

        return [
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]

    def launch_setup(self, context, *args, **kwargs):
        """启动设置函数"""
        actions = []

        # 获取配置文件路径
        config_file = LaunchConfiguration("config_file").perform(context)

        # 加载配置，获取实际加载路径
        resolved_path = str(Path(config_file).expanduser().resolve())
        config_data, is_valid = self.load_robot_config(config_file)

        # 解析机械臂配置
        enabled_arms, joint_names = self.parse_arms_config(config_data)

        if enabled_arms:
            actions.append(
                LogInfo(
                    msg=f"启用的机械臂: {', '.join(enabled_arms)} (配置: {resolved_path})"
                )
            )
        else:
            actions.append(LogInfo(msg=f"配置文件中没有启用的机械臂: {resolved_path}"))

        # 创建节点
        nodes = self.create_nodes(context, joint_names, config_file)
        actions.extend(nodes)

        return actions


def generate_launch_description():
    """生成启动描述"""
    launcher = ZlabRobotRvizLauncher()

    return LaunchDescription(
        [
            *launcher.get_launch_arguments(),
            OpaqueFunction(function=launcher.launch_setup),
        ]
    )
