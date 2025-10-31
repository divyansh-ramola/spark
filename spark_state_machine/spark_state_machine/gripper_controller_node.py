#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
import serial
import time
import threading

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('ur10e_enabled', True)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.ur10e_enabled = self.get_parameter('ur10e_enabled').get_parameter_value().bool_value
        
        # Initialize serial connection
        self.serial_conn = None
        self.init_serial_connection()
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        self.sequence_status_pub = self.create_publisher(String, 'sequence_status', 10)
        
       
        
        # Subscriber for vision X coordinate
        self.vision_x_sub = self.create_subscription(
            Float32, 'vision_x_coordinate', self.vision_x_callback, 10)
        
        # Service for executing the main sequence
        self.execute_sequence_service = self.create_service(
            Trigger, 'execute_main_sequence', self.execute_main_sequence_service)
        
        # UR10e command publishers
        if self.ur10e_enabled:
            self.ur10e_command_pub = self.create_publisher(String, 'ur10e_command', 10)
            self.ur10e_status_sub = self.create_subscription(
                String, 'ur10e_status', self.ur10e_status_callback, 10)
        
        # Current states
        self.current_x = 0.0
        self.ur10e_ready = False
        self.sequence_running = False
        
        # Lock for serial communication and sequence execution
        self.serial_lock = threading.Lock()
        self.sequence_lock = threading.Lock()
        
        self.get_logger().info('Arduino Controller Node Started')
        
        # Wait for Arduino to be ready
        self.wait_for_arduino_ready()

    def init_serial_connection(self):
        """Initialize serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.serial_conn = None

    def wait_for_arduino_ready(self):
        """Wait for Arduino to send READY signal"""
        if not self.serial_conn:
            self.get_logger().error('No serial connection available')
            return
            
        self.get_logger().info('Waiting for Arduino to be ready...')
        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode('utf-8').strip()
                    if response == 'READY':
                        self.get_logger().info('Arduino is ready!')
                        self.publish_status('ARDUINO_READY')
                        break
                    else:
                        self.get_logger().info(f'Arduino: {response}')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Error waiting for Arduino: {str(e)}')
                break

    def vision_x_callback(self, msg):
        """Callback for vision X coordinate"""
        self.current_x = msg.data
        self.get_logger().info(f'Received X coordinate from vision: {self.current_x}')

    def ur10e_status_callback(self, msg):
        """Callback for UR10e status updates"""
        status = msg.data
        self.get_logger().info(f'UR10e status: {status}')
        
        # Update UR10e ready state based on status
        if status == 'READY' or status.endswith('_COMPLETE'):
            self.ur10e_ready = True
        elif status.startswith('ERROR'):
            self.ur10e_ready = False

    def execute_main_sequence_service(self, request, response):
        """Service callback for executing the main sequence"""
        if self.sequence_running:
            response.success = False
            response.message = "Sequence is already running"
            return response
        
        # Execute in separate thread to avoid blocking the service call
        sequence_thread = threading.Thread(target=self._execute_main_sequence)
        sequence_thread.start()
        
        response.success = True
        response.message = "Main sequence started"
        return response

    def _execute_main_sequence(self):
        """Execute the main pick and place sequence"""
        with self.sequence_lock:
            self.sequence_running = True
            self.publish_sequence_status("MAIN_SEQUENCE_STARTED")
            
            try:
                self.get_logger().info("=== Starting Main Sequence ===")
                
                # Step 1: Home all systems
                self.get_logger().info("Step 1: Homing all systems")
                if not self.send_and_wait_for_completion('HOME', 'HOME_COMPLETE'):
                    raise Exception("Arduino home failed")
                
                # UR10e Home command (modify as needed for your UR10e controller)
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('HOME'):
                        raise Exception("UR10e home failed")
                
                # Step 2: UR10e moves to pick approach position
                self.get_logger().info("Step 2: UR10e moving to pick approach position")
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('MOVE_TO_PICK_APPROACH'):
                        raise Exception("UR10e move to pick approach failed")
                
                # Step 3: Arduino moves to target X position based on vision
                self.get_logger().info(f"Step 3: Arduino moving to X position: {self.current_x}")
                move_command = f'MOVE:{self.current_x}'
                if not self.send_and_wait_for_completion(move_command, 'MOVE_COMPLETE'):
                    raise Exception("Arduino move to X position failed")
                
                # Step 4: UR10e moves to precise pick position
                self.get_logger().info("Step 4: UR10e moving to precise pick position")
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('MOVE_TO_PICK_POSITION'):
                        raise Exception("UR10e move to pick position failed")
                
                # Step 5: Open gripper and lower servo to approach object
                self.get_logger().info("Step 5: Preparing for pick - opening gripper and lowering servo")
                if not self.send_and_wait_for_completion('GRIPPER_OPEN', 'GRIPPER_OPEN_COMPLETE'):
                    raise Exception("Gripper open failed")
                
                if not self.send_and_wait_for_completion('SERVO_CLOSE', 'SERVO_CLOSE_COMPLETE'):
                    raise Exception("Servo close (lower) failed")
                
                # Step 6: Close gripper to grab object
                self.get_logger().info("Step 6: Gripping object")
                if not self.send_and_wait_for_completion('GRIPPER_CLOSE', 'GRIPPER_CLOSE_COMPLETE'):
                    raise Exception("Gripper close failed")
                
                # Step 7: Raise servo to lift object
                self.get_logger().info("Step 7: Lifting object")
                if not self.send_and_wait_for_completion('SERVO_OPEN', 'SERVO_OPEN_COMPLETE'):
                    raise Exception("Servo open (raise) failed")
                
                # Step 8: UR10e moves to place approach position
                self.get_logger().info("Step 8: UR10e moving to place approach position")
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('MOVE_TO_PLACE_APPROACH'):
                        raise Exception("UR10e move to place approach failed")
                
                # Step 9: UR10e moves to precise place position
                self.get_logger().info("Step 9: UR10e moving to precise place position")
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('MOVE_TO_PLACE_POSITION'):
                        raise Exception("UR10e move to place position failed")
                
                # Step 10: Lower servo and release object
                self.get_logger().info("Step 10: Placing object")
                if not self.send_and_wait_for_completion('SERVO_CLOSE', 'SERVO_CLOSE_COMPLETE'):
                    raise Exception("Servo close (lower for place) failed")
                
                if not self.send_and_wait_for_completion('GRIPPER_OPEN', 'GRIPPER_OPEN_COMPLETE'):
                    raise Exception("Gripper open (release) failed")
                
                # Step 11: Raise servo to clear object
                self.get_logger().info("Step 11: Clearing object")
                if not self.send_and_wait_for_completion('SERVO_OPEN', 'SERVO_OPEN_COMPLETE'):
                    raise Exception("Servo open (clear) failed")
                
                # Step 12: UR10e moves away from place position
                self.get_logger().info("Step 12: UR10e moving away from place position")
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('MOVE_AWAY_FROM_PLACE'):
                        raise Exception("UR10e move away failed")
                
                # Step 13: Return all systems to home positions
                self.get_logger().info("Step 13: Returning all systems to home")
                if not self.send_and_wait_for_completion('HOME', 'HOME_COMPLETE'):
                    raise Exception("Arduino return home failed")
                
                if self.ur10e_enabled:
                    if not self.send_ur10e_command('HOME'):
                        raise Exception("UR10e return home failed")
                
                self.get_logger().info("=== Main Sequence Completed Successfully ===")
                self.publish_sequence_status("MAIN_SEQUENCE_COMPLETE")
                
            except Exception as e:
                self.get_logger().error(f"Main sequence failed: {str(e)}")
                self.publish_sequence_status(f"MAIN_SEQUENCE_FAILED: {str(e)}")
                
                # Emergency stop - try to home systems
                self.get_logger().info("Attempting emergency home...")
                try:
                    self.send_and_wait_for_completion('HOME', 'HOME_COMPLETE')
                    if self.ur10e_enabled:
                        self.send_ur10e_command('HOME')
                except:
                    self.get_logger().error("Emergency home failed")
            
            finally:
                self.sequence_running = False

    def send_ur10e_command(self, command):
        """Send command to UR10e robot and wait for completion"""
        if not self.ur10e_enabled:
            self.get_logger().warn("UR10e is disabled, skipping command")
            return True
        
        self.get_logger().info(f'Sending UR10e command: {command}')
        msg = String()
        msg.data = command
        self.ur10e_command_pub.publish(msg)
        
        # Wait for UR10e to complete command
        return self.wait_for_ur10e_completion(command)

    def wait_for_ur10e_completion(self, command, timeout=30.0):
        """Wait for UR10e command completion"""
        if not self.ur10e_enabled:
            return True
        
        self.ur10e_ready = False
        start_time = time.time()
        
        while time.time() - start_time < timeout and rclpy.ok():
            if self.ur10e_ready:
                self.get_logger().info(f'UR10e command {command} completed')
                return True
            time.sleep(0.1)
        
        self.get_logger().error(f'Timeout waiting for UR10e command: {command}')
        return False

    def execute_command(self, command):
        """Execute individual Arduino command"""
        if not self.serial_conn:
            self.get_logger().error('No serial connection available')
            self.publish_status('ERROR:NO_SERIAL_CONNECTION')
            return

        with self.serial_lock:
            try:
                # Handle different command types
                if command == 'HOME':
                    self.send_and_wait_for_completion('HOME', 'HOME_COMPLETE')
                
                elif command == 'MOVE':
                    move_command = f'MOVE:{self.current_x}'
                    self.send_and_wait_for_completion(move_command, 'MOVE_COMPLETE')
                
                elif command == 'GRIPPER_OPEN':
                    self.send_and_wait_for_completion('GRIPPER_OPEN', 'GRIPPER_OPEN_COMPLETE')
                
                elif command == 'GRIPPER_CLOSE':
                    self.send_and_wait_for_completion('GRIPPER_CLOSE', 'GRIPPER_CLOSE_COMPLETE')
                
                elif command == 'SERVO_OPEN':
                    self.send_and_wait_for_completion('SERVO_OPEN', 'SERVO_OPEN_COMPLETE')
                
                elif command == 'SERVO_CLOSE':
                    self.send_and_wait_for_completion('SERVO_CLOSE', 'SERVO_CLOSE_COMPLETE')
                
                elif command == 'INIT_SEQUENCE':
                    self.send_and_wait_for_completion('INIT_SEQUENCE', 'INIT_SEQUENCE_COMPLETE')
                
                elif command == 'FULL_PICK_SEQUENCE':
                    self.execute_pick_sequence()
                
                elif command == 'FULL_PLACE_SEQUENCE':
                    self.execute_place_sequence()
                
                else:
                    self.get_logger().warn(f'Unknown command: {command}')
                    self.publish_status(f'ERROR:UNKNOWN_COMMAND:{command}')

            except Exception as e:
                self.get_logger().error(f'Error executing command {command}: {str(e)}')
                self.publish_status(f'ERROR:EXECUTION_FAILED:{command}')

    def send_and_wait_for_completion(self, command, expected_completion):
        """Send command to Arduino and wait for completion"""
        self.get_logger().info(f'Sending Arduino command: {command}')
        
        # Send command
        self.serial_conn.write((command + '\n').encode('utf-8'))
        self.serial_conn.flush()
        
        # Wait for response
        start_time = time.time()
        timeout = 30.0  # 30 second timeout
        
        while time.time() - start_time < timeout:
            try:
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode('utf-8').strip()
                    self.get_logger().info(f'Arduino response: {response}')
                    
                    # Publish all Arduino responses as status
                    self.publish_status(response)
                    
                    # Check if command completed
                    if response == expected_completion:
                        self.get_logger().info(f'Arduino command {command} completed successfully')
                        return True
                    
                    # Check for errors
                    if response.startswith('ERROR'):
                        self.get_logger().error(f'Arduino error: {response}')
                        return False
                
                time.sleep(0.01)  # Small delay to prevent CPU overuse
                
            except Exception as e:
                self.get_logger().error(f'Error reading from Arduino: {str(e)}')
                return False
        
        self.get_logger().error(f'Timeout waiting for Arduino command {command} completion')
        self.publish_status(f'ERROR:TIMEOUT:{command}')
        return False

    def execute_pick_sequence(self):
        """Execute complete pick sequence (legacy support)"""
        self.get_logger().info('Executing pick sequence...')
        
        sequence_steps = [
            ('HOME', 'HOME_COMPLETE'),
            (f'MOVE:{self.current_x}', 'MOVE_COMPLETE'),
            ('GRIPPER_OPEN', 'GRIPPER_OPEN_COMPLETE'),
            ('SERVO_CLOSE', 'SERVO_CLOSE_COMPLETE'),
            ('GRIPPER_CLOSE', 'GRIPPER_CLOSE_COMPLETE')
        ]
        
        for i, (cmd, expected) in enumerate(sequence_steps):
            self.get_logger().info(f'Pick sequence step {i+1}/{len(sequence_steps)}: {cmd}')
            if not self.send_and_wait_for_completion(cmd, expected):
                self.publish_status(f'ERROR:PICK_SEQUENCE_FAILED_AT_STEP_{i+1}')
                return False
        
        self.get_logger().info('Pick sequence completed successfully')
        self.publish_status('PICK_SEQUENCE_COMPLETE')
        return True

    def execute_place_sequence(self):
        """Execute complete place sequence (legacy support)"""
        self.get_logger().info('Executing place sequence...')
        
        sequence_steps = [
            ('SERVO_OPEN', 'SERVO_OPEN_COMPLETE'),
            ('GRIPPER_OPEN', 'GRIPPER_OPEN_COMPLETE'),
            ('HOME', 'HOME_COMPLETE')
        ]
        
        for i, (cmd, expected) in enumerate(sequence_steps):
            self.get_logger().info(f'Place sequence step {i+1}/{len(sequence_steps)}: {cmd}')
            if not self.send_and_wait_for_completion(cmd, expected):
                self.publish_status(f'ERROR:PLACE_SEQUENCE_FAILED_AT_STEP_{i+1}')
                return False
        
        self.get_logger().info('Place sequence completed successfully')
        self.publish_status('PLACE_SEQUENCE_COMPLETE')
        return True

    def publish_status(self, status):
        """Publish Arduino status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_sequence_status(self, status):
        """Publish sequence status message"""
        msg = String()
        msg.data = status
        self.sequence_status_pub.publish(msg)
        self.get_logger().info(f'Sequence status: {status}')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    arduino_controller = ArduinoController()
    
    try:
        rclpy.spin(arduino_controller)
    except KeyboardInterrupt:
        pass
    
    arduino_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()