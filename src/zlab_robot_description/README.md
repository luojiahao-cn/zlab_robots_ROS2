# ZLab Robot Description

ZLab机械臂的ROS 2描述包，包含机器人模型定义和可视化工具。

## 功能特性

- 支持单机械臂和双机械臂配置
- 基于xacro的参数化URDF模型
- RViz可视化启动文件
- 自动化URDF生成脚本

## 快速使用

### 启动RViz可视化

```bash
# 使用默认配置
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py

# 使用单机械臂配置
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py config_file:=src/zlab_robot_description/config/robot_config_single_arm.yaml

# 使用双机械臂配置
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py config_file:=src/zlab_robot_description/config/robot_config_dual_arm.yaml
```

### 生成URDF文件

```bash
# 生成单机械臂URDF
./src/zlab_robot_description/launch/generate_urdf.sh -p single

# 生成双机械臂URDF
./src/zlab_robot_description/launch/generate_urdf.sh -p dual
```

## 文件结构

```
zlab_robot_description/
├── config/                     # 配置文件
│   ├── robot_config_default.yaml
│   ├── robot_config_single_arm.yaml
│   ├── robot_config_dual_arm.yaml
│   └── view_robot.rviz
├── launch/                     # 启动文件
│   ├── zlab_robot_rviz.launch.py
│   └── generate_urdf.sh
├── meshes/                     # 3D模型文件
│   ├── visual/
│   └── collision/
└── urdf/                       # URDF模型文件
    └── zlab_robot.urdf.xacro
```

## 配置说明

通过修改`config`目录下的YAML文件可以调整机械臂配置：

- `robot_config_default.yaml` - 默认配置
- `robot_config_single_arm.yaml` - 单机械臂配置
- `robot_config_dual_arm.yaml` - 双机械臂配置

## 依赖要求

- ROS 2 Jazzy
- xacro
- joint_state_publisher
- robot_state_publisher
- rviz2
