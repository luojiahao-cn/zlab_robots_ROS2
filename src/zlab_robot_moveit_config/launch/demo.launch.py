from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch argument
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="single",
        description="Type of arm configuration: 'single' or 'dual'",
    )
    
    return LaunchDescription([
        arm_type_arg,
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    arm_type = LaunchConfiguration("arm_type").perform(context)
    
    # Validate arm_type
    if arm_type not in ["single", "dual"]:
        raise ValueError(f"Invalid arm_type: {arm_type}. Must be 'single' or 'dual'")
    
    # Include the appropriate launch file
    demo_launch_file = f"demo_{arm_type}_arm.launch.py"
    
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("zlab_robot_moveit_config"),
                "launch",
                demo_launch_file
            ])
        ])
    )
    
    return [demo_launch]
