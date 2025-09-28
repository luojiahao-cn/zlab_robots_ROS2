# ZLab Robots ROS 2 Workspace

这个仓库包含 `zlab_robot_description` 及相关 ROS 2 包，用于在 ROS 2 Jazzy 环境下可视化与测试 ZLab 机械臂。

## 环境要求

- Ubuntu 24.04 或兼容 Linux 系统
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) 已正确安装，并完成 `rosdep`, `colcon` 等工具配置
- 可选：`vcstool`、`rosdep` 用于依赖管理

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/luojiahao-cn/zlab_robots_ROS2.git
cd zlab_robots_ROS2

# 安装依赖（可选，如果添加了新的依赖）
rosdep install --from-paths src --ignore-src -r -y

# 构建（忽略 build/install/log 将由 .gitignore 处理）
colcon build

# 加载工作空间环境
source install/setup.bash

# 启动 RViz 并加载机器人模型
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py
```

## 包说明

- `zlab_robot_description`：包含 URDF/Xacro、网格模型以及用于 RViz 的启动文件。
- 其余目录（`build/`, `install/`, `log/`）由 `colcon build` 自动生成，已经通过 `.gitignore` 排除，不需要提交。

## 常见操作

- 清理构建结果：

  ```bash
  rm -rf build install log
  ```


