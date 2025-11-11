#!/usr/bin/env python3

"""
Example script showing how to call the Arduino command service.

Usage:
    ros2 run spark_stage_2_ur arduino_service_client.py <command>
    
Commands:
    h - Home motors
    f - Find metal
    m - Move M0 away
    n - Move M1 away
"""

import sys
import rclpy
from rclpy.node import Node
from spark_stage_2_ur.srv import ArduinoCommand


class ArduinoServiceClient(Node):
    def __init__(self):
        super().__init__('arduino_service_client')
        self.client = self.create_client(ArduinoCommand, 'arduino/send_command')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arduino/send_command service...')
    
    def send_command(self, command):
        """Send a command to the Arduino"""
        request = ArduinoCommand.Request()
        request.command = command
        
        self.get_logger().info(f"Sending command '{command}' to Arduino...")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ Success: {response.message}")
            else:
                self.get_logger().error(f"✗ Failed: {response.message}")
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: arduino_service_client.py <command>")
        print("\nCommands:")
        print("  h - Home motors")
        print("  f - Find metal")
        print("  m - Move M0 away")
        print("  n - Move M1 away")
        sys.exit(1)
    
    command = sys.argv[1]
    
    # Validate command
    valid_commands = ['h', 'f', 'm', 'n']
    if command not in valid_commands:
        print(f"Error: Invalid command '{command}'")
        print(f"Valid commands are: {', '.join(valid_commands)}")
        sys.exit(1)
    
    node = ArduinoServiceClient()
    success = node.send_command(command)
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
