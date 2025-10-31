# Launch file that combines UR driver with ur10_e_moveit_config
# This ensures both use compatible configurations

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Use LaunchConfiguration substitutions directly (avoid early string resolution)
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tf_prefix = LaunchConfiguration("tf_prefix")
    prefix = LaunchConfiguration("prefix")
    
    # Launch UR driver without move_group (we'll use our own)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": "false",  # Disable UR driver's RViz
            "use_fake_hardware": use_fake_hardware,
            "tf_prefix": tf_prefix,
        }.items(),
    )
    
    # Launch our standalone MoveIt config
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur10_e_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        ]),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "launch_servo": "false",
            # Ensure MoveIt uses the same joint prefix
            "prefix": prefix,
        }.items(),
    )
    
    return [ur_control_launch, moveit_launch]


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Run UR driver with fake hardware (no real robot needed).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="TF/Joint name prefix for the UR driver; must match MoveIt prefix (empty to match default MoveIt config).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix for MoveIt; should match tf_prefix for UR driver.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
