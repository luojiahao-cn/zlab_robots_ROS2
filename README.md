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

# 构建
colcon build

# 加载工作空间环境
source install/setup.bash

# 启动 RViz 并加载机器人模型
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py

# 覆盖默认配置示例（可选）
ros2 launch zlab_robot_description zlab_robot_rviz.launch.py \
  config_file:=/absolute/path/to/zlab_robots_config.yaml \
  joint_state_gui:=false
```

## 包说明

- `zlab_robot_description`：包含 URDF/Xacro、网格模型以及用于 RViz 的启动文件。
- `zlab_robot_bringup`：包含配置文件和启动脚本，用于系统集成。
- `zlab_robot_moveit_config`：MoveIt! 配置包，包含运动规划相关配置。
- 其余目录（`build/`, `install/`, `log/`）由 `colcon build` 自动生成，已经通过 `.gitignore` 排除，不需要提交。

## 多臂与命名前缀配置

- 主配置文件：`src/zlab_robot_bringup/config/zlab_robots_config.yaml`
  - `arms.armX.enabled` 控制机械臂实例化，支持独立前缀（如 `l_` / `r_`）。
  - `base_xyz`/`base_rpy` 定义基座位姿，`dof` 用于生成关节列表。
  - `controllers` 段落指定每个 arm 对应的控制器名称，供 MoveIt / ros2_control 使用。
- 构建时使用 `colcon build --symlink-install`，这样安装目录下的 `share/zlab_robot_bringup/config/zlab_robots_config.yaml` 会指向源文件；修改源码中的 YAML 即可立即生效，若需要完全独立的配置，可通过 `config_file` 参数传入绝对路径。
- 启动文件新增参数：
  - `config_file`：显式指定 YAML 配置路径，便于使用外部覆盖文件。
  - `default_prefix`：当 YAML 中没有启用任何机械臂时的回退前缀。
  - `joint_state_gui` 默认开启，可在多臂场景下关闭以避免滑块过多。
- 运行时，`zlab_robot_rviz.launch.py` 会自动解析 YAML，按需生成关节名称，并传入 Xacro。

## MoveIt 与控制器参考配置

- `src/zlab_robot_bringup/config/` 目录包含各种配置文件：
  - `zlab_robots_config.yaml`：主配置文件（双臂）
  - `single_left_arm_config.yaml`：单左臂配置
  - `single_right_arm_config.yaml`：单右臂配置
- MoveIt! 示例文件未随包发布。如需集成 MoveIt!，请在你的工作区创建 `config/moveit/` 目录，并根据 `docs/multi_arm_configuration.md` 中的说明编写 `moveit_controllers.yaml`、`kinematics.yaml` 与 `planning.yaml`。

更多使用说明见 `docs/multi_arm_configuration.md`。

## 常见操作

- 清理构建结果：

  ```bash
  rm -rf build install log
  ```


