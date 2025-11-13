#!/usr/bin/env python3
"""
Example script showing how to send joint angles directly to the robot.

Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
All angles are in radians.
"""

import rclpy
from rclpy.node import Node
from spark_stage_2_ur.srv import MoveToJoints
import math


class JointMoveExample(Node):
    def __init__(self):
        super().__init__('joint_move_example')
        self.client = self.create_client(MoveToJoints, 'move_to_joints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_to_joints service...')
        
        self.get_logger().info('Service is available!')

    def move_to_joints(self, joint_angles, description=""):
        """
        Send joint angles to the robot.
        
        Args:
            joint_angles: List of 6 joint angles in radians
            description: Optional description for logging
        """
        request = MoveToJoints.Request()
        request.joint_positions = joint_angles
        
        self.get_logger().info(f'Sending joint angles{": " + description if description else ""}')
        self.get_logger().info(f'  Angles (rad): {joint_angles}')
        self.get_logger().info(f'  Angles (deg): {[math.degrees(a) for a in joint_angles]}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'✓ Success: {response.message}')
        else:
            self.get_logger().error(f'✗ Failed: {response.message}')
        
        return response.success


def main(args=None):
    rclpy.init(args=args)
    node = JointMoveExample()
    
    # Example 1: Home position (all zeros)
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.move_to_joints(home_position, "Home position")
    
    # Wait a moment
    import time
    time.sleep(2)
    
    # Example 2: Custom position (example values in radians)
    # Note: Adjust these values based on your robot's workspace and limits
    custom_position = [
        0.0,           # shoulder_pan: 0 degrees
        -math.pi/4,    # shoulder_lift: -45 degrees
        math.pi/2,     # elbow: 90 degrees
        -math.pi/4,    # wrist_1: -45 degrees
        0.0,           # wrist_2: 0 degrees
        0.0            # wrist_3: 0 degrees
    ]
    node.move_to_joints(custom_position, "Custom position")
    
    # Example 3: Move with specific joint angles in degrees (converted to radians)
    def deg_to_rad(degrees_list):
        return [math.radians(deg) for deg in degrees_list]
    
    # Define position in degrees for easier reading
    position_in_degrees = [0, -90, 90, -90, 0, 0]
    position_in_radians = deg_to_rad(position_in_degrees)
    
    time.sleep(2)
    node.move_to_joints(position_in_radians, f"Position: {position_in_degrees} degrees")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
