from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package=package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Paths to config files (reuse from ur10_e_moveit_config)
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

    # SRDF (static file in ur10_e_moveit_config)
    srdf_path = PathJoinSubstitution([FindPackageShare("ur10_e_moveit_config"), "srdf", "ur10e.srdf"])
    with open(srdf_path.perform(context), 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Kinematics and planning params
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur10_e_moveit_config"), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": load_yaml("ur10_e_moveit_config", "config/joint_limits.yaml")
    }

    # Include MoveIt move_group launch (brings up action server)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur10_e_moveit_config"), "launch", "move_group.launch.py"]) 
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # pass through same URDF file and prefix/ip if needed
            "description_file": description_file,
            "prefix": prefix,
            "robot_ip": robot_ip,
            # allow toggling RViz from this wrapper
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # Our application node, with all needed MoveIt params
    move_to_pose_node = Node(
        package="spark_stage_2_ur",
        executable="move_to_pose_node",
        name="move_to_pose_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group_launch, move_to_pose_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file", default_value="ur10e.urdf.xacro", description="URDF/XACRO description file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", default_value='""', description="Joint name prefix")
    )
    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", default_value="xxx.yyy.zzz.www", description="UR controller IP")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz as part of MoveIt stack")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
