#!/bin/bash
# URDF生成脚本 - 支持单臂和双臂配置

# 设置默认参数
CONFIG_FILE=""
OUTPUT_FILE=""
PRESET=""

# 帮助函数
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -p, --preset   预设配置 (single|dual)"
    echo "  -c, --config   自定义配置文件路径"
    echo "  -o, --output   输出文件路径"
    echo "  -h, --help     显示此帮助信息"
    echo ""
    echo "预设配置:"
    echo "  single         单机械臂 (输出: src/zlab_robot_description/urdf/zlab_robot_single_arm.urdf)"
    echo "  dual           双机械臂 (输出: src/zlab_robot_description/urdf/zlab_robot_dual_arm.urdf)"
    echo ""
    echo "示例:"
    echo "  $0 -p single"
    echo "  $0 -p dual"
    echo "  $0 -c my_config.yaml -o my_robot.urdf"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--preset)
            PRESET="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 确保我们在正确的工作目录
cd /home/lawkaho/workshop/zlab_robots_ROS2

# 根据预设设置配置文件和输出文件
if [ ! -z "$PRESET" ]; then
    case $PRESET in
        single)
            CONFIG_FILE="$(pwd)/src/zlab_robot_description/config/robot_config_single_arm.yaml"
            if [ -z "$OUTPUT_FILE" ]; then
                OUTPUT_FILE="src/zlab_robot_description/urdf/zlab_robot_single_arm.urdf"
            fi
            ;;
        dual)
            CONFIG_FILE="$(pwd)/src/zlab_robot_description/config/robot_config_dual_arm.yaml"
            if [ -z "$OUTPUT_FILE" ]; then
                OUTPUT_FILE="src/zlab_robot_description/urdf/zlab_robot_dual_arm.urdf"
            fi
            ;;
        *)
            echo "错误: 未知的预设配置 '$PRESET'"
            echo "可用预设: single, dual"
            exit 1
            ;;
    esac
fi

# 如果没有指定配置文件，使用默认配置
if [ -z "$CONFIG_FILE" ]; then
    echo "正在使用默认配置生成URDF..."
    if [ -z "$OUTPUT_FILE" ]; then
        OUTPUT_FILE="src/zlab_robot_description/urdf/zlab_robot.urdf"
    fi
    # 确保输出目录存在
    mkdir -p "$(dirname "$OUTPUT_FILE")"
    ros2 run xacro xacro src/zlab_robot_description/urdf/zlab_robot.urdf.xacro > "$OUTPUT_FILE"
else
    echo "正在生成URDF..."
    echo "配置文件: $CONFIG_FILE"
    if [ -z "$OUTPUT_FILE" ]; then
        OUTPUT_FILE="src/zlab_robot_description/urdf/zlab_robot.urdf"
    fi
    # 确保输出目录存在
    mkdir -p "$(dirname "$OUTPUT_FILE")"
    ros2 run xacro xacro src/zlab_robot_description/urdf/zlab_robot.urdf.xacro config_file:="$CONFIG_FILE" > "$OUTPUT_FILE"
fi

echo "输出文件: $OUTPUT_FILE"

if [ $? -eq 0 ]; then
    echo "✓ URDF生成成功: $OUTPUT_FILE"
    echo "文件大小: $(wc -c < $OUTPUT_FILE) 字节"
else
    echo "✗ URDF生成失败"
    exit 1
fi