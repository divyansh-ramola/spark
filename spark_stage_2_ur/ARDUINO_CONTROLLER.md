# Arduino Motor Controller Node

This node provides ROS2 integration for Arduino-based motor control via serial communication.

## Service Interface

The node exposes a single service that accepts character commands:

**Service:** `arduino/send_command`  
**Type:** `spark_stage_2_ur/srv/ArduinoCommand`

### Service Definition

```
# Request
char command        # Command character (h, f, m, n)
---
# Response
bool success        # Whether command execution succeeded
string message      # Status or error message
```

## Supported Commands

| Command | Description |
|---------|-------------|
| `h` | Home motors |
| `f` | Find metal |
| `m` | Move M0 away |
| `n` | Move M1 away |

## Launch

```bash
# Launch with default parameters
ros2 launch spark_stage_2_ur arduino_controller.launch.py

# Launch with custom serial port
ros2 launch spark_stage_2_ur arduino_controller.launch.py serial_port:=/dev/ttyACM0

# Launch with custom baud rate and timeout
ros2 launch spark_stage_2_ur arduino_controller.launch.py baud_rate:=115200 timeout_ms:=60000
```

## Parameters

- `serial_port` (default: `/dev/ttyUSB0`): Serial port for Arduino connection
- `baud_rate` (default: `9600`): Baud rate for serial communication
- `timeout_ms` (default: `30000`): Timeout in milliseconds for Arduino commands

## Usage Examples

### Using the Python Client

```bash
# Home motors
ros2 run spark_stage_2_ur arduino_service_client.py h

# Find metal
ros2 run spark_stage_2_ur arduino_service_client.py f

# Move M0 away
ros2 run spark_stage_2_ur arduino_service_client.py m

# Move M1 away
ros2 run spark_stage_2_ur arduino_service_client.py n
```

### Using Command Line

```bash
# Home motors
ros2 service call /arduino/send_command spark_stage_2_ur/srv/ArduinoCommand "{command: 'h'}"

# Find metal
ros2 service call /arduino/send_command spark_stage_2_ur/srv/ArduinoCommand "{command: 'f'}"

# Move M0 away
ros2 service call /arduino/send_command spark_stage_2_ur/srv/ArduinoCommand "{command: 'm'}"

# Move M1 away
ros2 service call /arduino/send_command spark_stage_2_ur/srv/ArduinoCommand "{command: 'n'}"
```

### Python Example

```python
import rclpy
from rclpy.node import Node
from spark_stage_2_ur.srv import ArduinoCommand

rclpy.init()
node = Node('my_node')
client = node.create_client(ArduinoCommand, 'arduino/send_command')

# Wait for service
client.wait_for_service()

# Send command
request = ArduinoCommand.Request()
request.command = 'h'  # Home motors
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

response = future.result()
if response.success:
    print(f"Success: {response.message}")
else:
    print(f"Failed: {response.message}")
```

### C++ Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include "spark_stage_2_ur/srv/arduino_command.hpp"

auto node = std::make_shared<rclcpp::Node>("my_node");
auto client = node->create_client<spark_stage_2_ur::srv::ArduinoCommand>("arduino/send_command");

// Wait for service
client->wait_for_service();

// Send command
auto request = std::make_shared<spark_stage_2_ur::srv::ArduinoCommand::Request>();
request->command = 'h';  // Home motors

auto future = client->async_send_request(request);
if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(node->get_logger(), "Success: %s", response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed: %s", response->message.c_str());
    }
}
```

## Topics

**Published:**
- `arduino/status` (`std_msgs/String`): Status messages and logs from Arduino

## Arduino Protocol

The node expects the following serial protocol from the Arduino:

### Arduino → ROS
- `READY` - Arduino is ready to receive commands
- `ACK:<command>` - Command received and acknowledged
- `DONE:<command>` - Command completed successfully
- `ERROR:<command>` - Command failed

### ROS → Arduino
Single character commands: `h`, `f`, `m`, `n`

## Building

```bash
cd ~/spark_ws
colcon build --packages-select spark_stage_2_ur
source install/setup.bash
```

## Dependencies

- ROS2 Humble
- rclcpp
- std_msgs
- std_srvs
- serial library (`libserial-dev`)

Install serial library if needed:
```bash
sudo apt-get install libserial-dev
```
