


import time
import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient

# Import custom service and message types
from spark_state_machine_interfaces.srv import MoveGripper
from spark_state_machine_interfaces.msg import GripperStatus

# Gripper Modbus parameters from the manual
BAUD_RATE = 115200
REG_INITIALIZE = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_STATUS = 0x0201


#ros2 service call /move_gripper dh_gripper_driver/srv/MoveGripper "{target_position: 500, target_force: 30}"

class DhGripperNode(Node):
    """A ROS2 node to control the DH-Robotics PGE Gripper via Modbus RTU."""

    def __init__(self):
        super().__init__('dh_gripper_node')

        # Declare parameters for port and device ID
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('device_id', 1)

        port = self.get_parameter('port').get_parameter_value().string_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value

        # --- Initialize Modbus Client ---
        self.client = ModbusSerialClient(
            port=port,
            baudrate=BAUD_RATE,
            stopbits=1,
            bytesize=8,
            parity='N',
            timeout=1
        )
        self.get_logger().info(f"Connecting to gripper on port {port} with device ID {self.device_id}...")
        if not self.client.connect():
            self.get_logger().error("Failed to connect to the gripper. Shutting down.")
            # This is a critical failure, so we'll destroy the node.
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info("Connection successful. Initializing gripper...")
        self.initialize_gripper()

        # --- Create ROS2 Service, Publisher, and Timer ---
        self.move_service = self.create_service(MoveGripper, 'move_gripper', self.move_gripper_callback)
        self.status_publisher = self.create_publisher(GripperStatus, 'gripper_status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status_callback) # Publish status every 1 second
        
        # Ensure connection is closed on shutdown
        self.context.on_shutdown(self.on_shutdown)

    def initialize_gripper(self):
        """Initializes the gripper. Must be done before any other command."""
        try:
            # pymodbus v3+ uses `device_id=` instead of deprecated `slave`
            self.client.write_register(REG_INITIALIZE, 1, device_id=self.device_id)
            time.sleep(3) # Wait for initialization to complete
            self.get_logger().info("Gripper initialization complete.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize gripper: {e}")

    def move_gripper_callback(self, request, response):
        """Callback for the move_gripper service."""
        pos = request.target_position
        force = request.target_force
        
        self.get_logger().info(f"Received request: move to position {pos} with force {force}")

        # Validate inputs
        if not 0 <= pos <= 1000:
            response.success = False
            response.message = "Error: Position must be between 0 and 1000."
            self.get_logger().error(response.message)
            return response
        if not 20 <= force <= 100:
            response.success = False
            response.message = "Error: Force must be between 20 and 100."
            self.get_logger().error(response.message)
            return response
            
        try:
            # Set force first, then position
            self.client.write_register(REG_FORCE, int(force), device_id=self.device_id)
            self.client.write_register(REG_POSITION, int(pos), device_id=self.device_id)
            
            response.success = True
            response.message = f"Successfully set position to {pos} and force to {force}."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to send commands to gripper: {e}"
            self.get_logger().error(response.message)

        return response

    def publish_status_callback(self):
        """Periodically reads and publishes the gripper's status."""
        try:
            read_response = self.client.read_holding_registers(REG_STATUS, device_id=self.device_id)
            if read_response.isError():
                self.get_logger().warn("Error reading gripper status.")
                return

            status_code = read_response.registers[0]
            status_map = {
                0: "In motion",
                1: "Reached position (no object detected)",
                2: "Object caught",
                3: "Object dropped"
            }
            status_text = status_map.get(status_code, "Unknown status code")

            msg = GripperStatus()
            msg.status_code = status_code
            msg.status_text = status_text
            self.status_publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not publish status: {e}")

    def on_shutdown(self):
        """Closes the serial connection when the node is shut down."""
        self.get_logger().info("Closing Modbus connection.")
        self.client.close()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DhGripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()