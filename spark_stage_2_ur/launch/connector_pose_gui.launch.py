from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    connector_gui = Node(
        package='spark_stage_2_ur',
        executable='connector_pose_gui.py',
        name='connector_pose_gui',
        output='screen',
    )
    
    return LaunchDescription([
        connector_gui,
    ])
