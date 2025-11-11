import launch
import os
import sys
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=10.42.0.86",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic


def kinematics_param_file():
    """Return a ParameterFile reference to the kinematics.yaml in ur10_e_moveit_config."""
    return ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("ur10_e_moveit_config"),
            "config",
            "kinematics.yaml",
        ])
    )
    
def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    
    # Robot State Publisher - subscribes to /joint_states and publishes TF transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # NOTE: This launch file expects /joint_states to be published by the UR driver
    # Start the UR driver first:
    #   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=10.42.0.86
    
    # Kinematics parameters for MoveIt (suppresses 'No kinematics plugins defined' warning)
    demo_node = Node(
        package="spark_stage_2_ur",
        executable=  "move_to_pose_node",  # bin_pick_node
        name=    "move_to_pose_node",  # bin_pick_node
        output="screen",
        arguments=["--ros-args", "--log-level", "move_group_interface:=warn"],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_param_file(),
        ],
    )

    return launch.LaunchDescription([
        robot_state_publisher,
        demo_node,
    ])