# zlab_robot_description

> ROS 2 机械臂模型描述包，提供配置化的 URDF 模型和 RViz 可视化。

## 项目概述

`zlab_robot_description` 是一个 ROS 2 机械臂描述包，主要功能：

- 基于 Xacro 构建的 URDF，支持多机械臂配置
- 通过 YAML 配置文件管理机械臂参数
- 提供 RViz 可视化启动文件
- 支持关节状态发布和控制

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble 或更新版本
- 必需依赖：`xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`

```bash
sudo apt install ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-rviz2 python3-yaml
```

## 快速使用

```bash
# 1. 克隆仓库
cd ~/ros2_ws/src
git clone https://github.com/luojiahao-cn/zlab_robots_ROS2.git

# 2. 构建包
cd ~/ros2_ws
colcon build --packages-select zlab_robot_description
source install/setup.bash

# 3. 启动 RViz 可视化
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py
```

## 配置文件

机械臂配置位于 `config/zlab_robots_config.yaml`：

```yaml
arms:
  arm1:
    arm_id: zlab_arm_left
    prefix: l_
    enabled: true
    base_xyz: [0.0, 0.0, 0.0]
    base_rpy: [0.0, 0.0, 0.0]
    dof: 6
  arm2:
    arm_id: zlab_arm_right  
    prefix: r_
    enabled: false
    base_xyz: [0.0, 0.8, 0.0]
    base_rpy: [0.0, 0.0, 0.0]
    dof: 6
```

### 主要参数

- `enabled`: 是否启用该机械臂
- `prefix`: 关节和链接的命名前缀
- `base_xyz/base_rpy`: 机械臂基座的位置和姿态
- `dof`: 自由度数量

## 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | 包内默认配置 | 配置文件路径 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `joint_state_gui` | `true` | 是否启用关节状态 GUI |

### 使用自定义配置

```bash
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py \
    config_file:=/path/to/your/config.yaml \
    joint_state_gui:=false
```

## 目录结构

```
zlab_robot_description/
├── config/          # 配置文件
├── launch/          # 启动文件
├── meshes/          # 3D 模型文件
└── urdf/            # URDF/Xacro 文件
```

## 常见问题

**Q: 提示配置文件不存在？**
A: 检查 `config_file` 参数路径是否正确，建议使用绝对路径。

**Q: RViz 中看不到模型？**
A: 确认配置文件中至少有一个机械臂的 `enabled: true`。

**Q: 关节控制器不工作？**
A: 检查是否安装了 `joint_state_publisher_gui` 包。

## 许可证

本项目采用 Apache-2.0 许可证发布。
