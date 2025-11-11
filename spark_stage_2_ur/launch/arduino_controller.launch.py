from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    timeout_ms_arg = DeclareLaunchArgument(
        'timeout_ms',
        default_value='30000',
        description='Timeout in milliseconds for Arduino commands'
    )
    
    # Arduino controller node
    arduino_controller_node = Node(
        package='spark_stage_2_ur',
        executable='arduino_controller_node',
        name='arduino_motor_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout_ms': LaunchConfiguration('timeout_ms'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        timeout_ms_arg,
        arduino_controller_node,
    ])
