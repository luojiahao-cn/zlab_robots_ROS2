# ZLab Robot MoveIt Configuration

这个包提供了 ZLab 机器人的 MoveIt 配置，支持单机械臂和双机械臂配置。

## 目录结构

```
zlab_robot_moveit_config/
├── config/
│   ├── single_arm/          # 单臂配置
│   │   └── zlab_robot.srdf
│   ├── dual_arm/            # 双臂配置
│   │   └── zlab_robot.srdf
│   ├── kinematics.yaml      # 运动学求解器配置
│   ├── joint_limits.yaml    # 关节限制配置
│   ├── controllers.yaml     # 控制器配置
│   ├── planning_pipeline.yaml  # 规划管道配置
│   └── ompl_planning.yaml   # OMPL 规划器配置
├── launch/
│   ├── demo_single_arm.launch.py  # 单臂演示启动文件
│   └── demo_dual_arm.launch.py    # 双臂演示启动文件
├── rviz/
│   └── moveit.rviz          # RViz 配置文件
└── package.xml
```

## 使用方法

### 1. 构建工作空间

```bash
cd /path/to/your/workspace
colcon build
source install/setup.bash
```

### 2. 启动单臂配置

```bash
ros2 launch zlab_robot_moveit_config demo_single_arm.launch.py
```

### 3. 启动双臂配置

```bash
ros2 launch zlab_robot_moveit_config demo_dual_arm.launch.py
```

## 配置说明

### 单臂配置 (single_arm)
- 规划组: `zlab_arm`
- 关节: `zlab_arm_j1` 到 `zlab_arm_j6`
- 末端执行器: `zlab_arm_tool0`

### 双臂配置 (dual_arm)
- 规划组:
  - `left_arm`: 左臂 (`l_j1` 到 `l_j6`)
  - `right_arm`: 右臂 (`r_j1` 到 `r_j6`)
  - `both_arms`: 双臂组合
- 末端执行器:
  - `l_tool0`: 左臂末端
  - `r_tool0`: 右臂末端

## 运动学求解器

使用 KDL (Kinematics and Dynamics Library) 作为运动学求解器，适用于 6DOF 机械臂。

## 规划器

支持多种 OMPL 规划器，包括 RRT、RRTConnect、RRT* 等。

## 注意事项

1. 确保已安装 MoveIt 和相关依赖
2. RViz 中需要选择正确的规划组进行规划
3. 控制器配置需要根据实际硬件进行调整