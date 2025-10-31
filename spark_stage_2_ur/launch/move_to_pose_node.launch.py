from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
import os


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_ip = LaunchConfiguration("robot_ip")

    # Paths to config files from ur10_e_moveit_config
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "ur10e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "ur10e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "ur10e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "ur10e", "visual_parameters.yaml"]
    )

    # Build robot_description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur10_e_moveit_config"), "urdf", description_file]),
            " ",
            "robot_ip:=", robot_ip, " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=true ",
            "safety_pos_margin:=0.15 ",
            "safety_k_position:=20 ",
            "name:=ur10e ",
            "ur_type:=ur10e ",
            "script_filename:=ros_control.urscript ",
            "input_recipe_filename:=rtde_input_recipe.txt ",
            "output_recipe_filename:=rtde_output_recipe.txt ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Build SRDF from xacro so prefix is applied consistently
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur10_e_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            "ur10e ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # Kinematics and planning params
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "kinematics.yaml"]
    )

    node = Node(
        package="spark_stage_2_ur",
        executable="move_to_pose_node",
        name="move_to_pose_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time")
    )
    declared_arguments.append(
        DeclareLaunchArgument("description_file", default_value="ur10e.urdf.xacro", description="URDF/XACRO file")
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", default_value='""', description="Joint name prefix")
    )
    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", default_value="xxx.yyy.zzz.www", description="UR controller IP")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
