# Wire Harnessing System Documentation - SPARK MINDA

## Repository Overview

This repository contains the complete ROS2-based control system for automated wire harnessing operations at SPARK MINDA. The system integrates UR10e robot arm, custom grippers, Arduino-controlled jigs, Intel RealSense camera, and a web-based GUI for monitoring and control.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Package Structure](#package-structure)
3. [Stage 1: Wire Straightening System](#stage-1-wire-straightening-system)
4. [Stage 2: Wire Insertion System (Current)](#stage-2-wire-insertion-system-current)
5. [Hardware Setup](#hardware-setup)
6. [Software Setup](#software-setup)
7. [System Bringup](#system-bringup)
8. [Programming Guide](#programming-guide)
9. [Troubleshooting](#troubleshooting)

---

## System Architecture

### Core Components

1. **UR10e Robot Arm**: 6-DOF industrial robot for precise positioning
2. **Custom EOAT (End-Of-Arm-Tooling)**: 
   - Stage 1: Wire straightening gripper
   - Stage 2: Multi-gripper assembly for connector insertion
3. **Arduino Controllers**: Custom jig control and wire detection
4. **Intel RealSense D435**: RGB-D camera for vision-based wire detection
5. **Web GUI**: FastAPI backend + React frontend for system monitoring and control

### Software Stack

- **ROS2 Humble**: Core middleware
- **MoveIt 2**: Motion planning framework
- **OpenCV + YOLOv8**: Computer vision and wire tip detection
- **FastAPI**: Backend REST API and WebSocket server
- **React**: Frontend web interface

---

## Package Structure

```
spark_ws/
├── src/
│   ├── spark_state_machine/           # Stage 1 - Wire Straightening
│   │   ├── arduino/                   # Arduino code for Stage 1
│   │   ├── spark_state_machine/       # Python nodes
│   │   │   ├── gripper_controller_node.py
│   │   │   ├── wire_tip_detection.py
│   │   │   └── wire_tip_detection_test.py
│   │   └── srv/                       # Custom service definitions
│   │
│   ├── spark_stage_2_ur/              # Stage 2 - Wire Insertion (CURRENT)
│   │   ├── arduino/                   # Arduino code for Stage 2
│   │   │   └── finalwithresponse/
│   │   │       └── sketch/            # Main Arduino sketch
│   │   ├── config/                    # MoveIt configuration
│   │   ├── launch/                    # Launch files
│   │   ├── src/                       # C++ nodes
│   │   │   ├── move_to_pose_node.cpp  # Main robot control node
│   │   │   └── Arduino_controller_node.cpp
│   │   ├── srv/                       # Service definitions
│   │   ├── connector_poses.yaml       # Saved connector positions
│   │   └── include/
│   │       └── spark_stage_2_ur/
│   │           └── connector_position.hpp
│   │
│   ├── spark_state_machine_interfaces/  # Custom message/service types
│   ├── ur10_e_moveit_config/            # Custom MoveIt config
│   ├── spark_minda_gui_backend/         # Web backend (FastAPI)
│   └── spark_minda_gui_frontend/        # Web frontend (React)
└── ex/                                  # External packages (UR, RealSense, etc.)
```

---

## Stage 1: Wire Straightening System

### Purpose
Automatically detect wire tips using vision, pick them up with a gripper, and straighten them for processing.

### Key Nodes

#### 1. `gripper_controller_node.py`
**Location**: `spark_state_machine/spark_state_machine/`

**Purpose**: Controls Arduino-based gripper system and coordinates pick-place operations

**Serial Port**: `/dev/ttyUSB0` (default, configurable)

**Topics**:
- Publisher: `arduino_status` (String) - Arduino status messages
- Publisher: `sequence_status` (String) - Sequence execution status
- Subscriber: `vision_x_coordinate` (Float32) - X position from vision
- Subscriber: `ur10e_status` (String) - UR10e robot status

**Services**:
- `execute_main_sequence` (Trigger) - Execute full pick-place sequence

**Arduino Commands**:
```
HOME           - Home all motors
MOVE:<x>       - Move to X position
GRIPPER_OPEN   - Open gripper
GRIPPER_CLOSE  - Close gripper
SERVO_OPEN     - Raise servo
SERVO_CLOSE    - Lower servo
```

**Usage**:
```bash
ros2 run spark_state_machine gripper_controller_node

# Execute pick-place sequence
ros2 service call /execute_main_sequence std_srvs/srv/Trigger
```

#### 2. `wire_tip_detection.py`
**Location**: `spark_state_machine/spark_state_machine/`

**Purpose**: YOLOv8-based wire tip detection and position calculation

**Camera**: Intel RealSense D435 (`/dev/ttyACM1` for Arduino)

**Model**: `best.pt` (YOLOv8 trained on wire tips)

**Services**:
- `execute_seq1` (Trigger) - Set X=0 and execute sequence
- `execute_seq2` (Trigger) - Open both grippers
- `toggle_relay` (Trigger) - Toggle relay state

**Topics**:
- Publisher: `vision_x_coordinate` (Float32) - Detected wire X position

**Usage**:
```bash
ros2 run spark_state_machine wire_tip_detection

# Test tip detection
ros2 service call /execute_seq1 std_srvs/srv/Trigger
```

#### 3. `wire_tip_detection_test.py`
Alternative wire detection node with different model/parameters for testing.

**Usage**:
```bash
ros2 run spark_state_machine wire_tip_detection_test

# Get tip X coordinate
ros2 service call /get_tip_x_coord std_srvs/srv/Trigger
```

### Arduino Code
**Location**: `spark_state_machine/arduino/`

Implements servo control, stepper motors, and gripper mechanisms for wire straightening operations.

---

## Stage 2: Wire Insertion System (Current)

### Purpose
Automated wire connector insertion into PCB jig positions with force feedback, vision guidance, and error recovery.

### System Workflow

```
1. Initialize System → Upload Arduino code, start nodes, launch GUI
2. Home Robot → Move to safe starting position
3. Wire Detection → Vacuum sensor detects wire presence
4. Wire Pickup → Grasp wire with gripper DO0
5. Move to Jig → Transport wire to insertion jig
6. Place in Jig → Insert wire into jig holder
7. Arduino Processing → 'a' (advance), 'f' (find metal), 'h' (home)
8. Tip Operations → Orient and grip connector tip
9. Slide to Position → Move to connector alignment position
10. Connector Insertion → Insert connector into specified (row, col)
11. Wire Quality Check → Publish pass/fail on /wire_quality
12. Final Positioning → Move to safe position, ready for next cycle
```

### Key Nodes

#### 1. `move_to_pose_node.cpp` ⭐ MAIN NODE
**Location**: `spark_stage_2_ur/src/`

**Purpose**: Core robot control, sequence orchestration, gripper management, and connector insertion

**Topics**:
- Publisher: `/process_status` (String) - Current process status
- Publisher: `/grid_position_update` (Int32) - Current grid position (0-23)
- Publisher: `/wire_quality` (Bool) - Wire quality (true=pass, false=fail)
- Subscriber: `/force_torque_sensor_broadcaster/wrench` - Force/torque readings
- Subscriber: `/io_and_status_controller/io_states` - Vacuum sensor analog input

**Services Provided**:
```cpp
/move_to_pose              - Execute connector sequence (Trigger)
/move_linear_quat          - Linear Cartesian move with quaternion
/move_to_joints            - Move to joint angles
/set_grid_positions        - Process multiple grid positions
```

**Service Clients**:
```cpp
/io_and_status_controller/set_io  - Control digital outputs (grippers)
arduino/send_command               - Send commands to Arduino
```

**Main Function: `run_connector_sequence(row, col)`**

This is the core sequence function. Modify steps here to change the operation flow:

```cpp
std::pair<bool, std::string> run_connector_sequence(const int& row, const int& col)
{
    // Helper lambda for automatic retry logic
    auto execute_with_retry = [this](const std::string& step_name, 
                                      std::function<bool()> operation) -> bool {
        // Attempts operation twice before failing
        // First attempt
        bool success = operation();
        if (success) return true;
        
        // Retry once
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        success = operation();
        return success;
    };

    // Step 1: Initial positioning
    if (!execute_with_retry("Step 1: Initial positioning", [&]() {
        auto [ok, msg] = movelinear(-0.295, -0.655, 0.836, 1.000, 0.023, -0.008, -0.008);
        return ok;
    })) {
        return {false, "Step 1: Initial positioning failed"};
    }

    // Step 2: Arduino setup and gripper initialization
    // ... (continues for all steps)
}
```

**Available Motion Commands**:

```cpp
// Linear Cartesian motion with quaternion orientation
std::pair<bool, std::string> movelinear(
    double x, double y, double z,        // Position (meters)
    double qx, double qy, double qz, double qw  // Orientation (quaternion)
);

// Relative linear motion (jog)
std::pair<bool, std::string> joglinear(
    double x_offset,   // X offset (meters)
    double y_offset,   // Y offset (meters)
    double z_offset    // Z offset (meters)
);

// Move to joint angles
std::pair<bool, std::string> movejoints(
    double j0,  // shoulder_pan_joint
    double j1,  // shoulder_lift_joint
    double j2,  // elbow_joint
    double j3,  // wrist_1_joint
    double j4,  // wrist_2_joint
    double j5   // wrist_3_joint
);
```

**Gripper Control Functions**:

```cpp
// Pneumatic grippers (3 independent grippers)
bool ngripper(bool state, int index);
// index: 0, 1, or 2
// state: true=close, false=open
// Pin mapping:
//   index=0: Pin4=close, Pin5=open
//   index=1: Pin1=close, Pin2=open
//   index=2: Pin6=close, Pin7=open

// Electric gripper (small/closed positions)
bool egripper(bool state);
// state: true=fully closed (both pins LOW)
//        false=small opening (Pin17 HIGH, Pin16 LOW)
// Pin 16 HIGH = Large opening (not used)
// Pin 17 HIGH = Small opening
// Both LOW = Fully closed

bool egripper2(bool state);
// Similar to egripper but controls Pin 16
```

**Arduino Commands** (sent via `send_arduino_command(char)`):

```cpp
'h' or 'H'  - Home Motors
'a' or 'A'  - Move M1 Forward (advance wire into jig)
'f' or 'F'  - Find Metal (detect wire ferrule)
'm' or 'M'  - Move M0 Away
'n' or 'N'  - Move M1 Away
'p' or 'P'  - Move M0 Towards Home
'q' or 'Q'  - Move M1 Towards Home
```

**Sensor Callbacks**:

```cpp
void vacuum_callback(IOStates msg)
// Monitors analog input from vacuum sensor
// Detects wire presence by voltage drop
// Threshold: (0.87-0.73)/4 ≈ 0.035V
// Sets atomic flag: vacuum_active_

void wrench_callback(WrenchStamped msg)
// Monitors force/torque sensor
// Detects high force (>50N) for collision detection
// Sets atomic flag: wrench_active_
```

**Error Recovery**:

The system implements automatic error recovery for metal detection failures:

```cpp
bool execute_find_metal_recovery()
// Executes when 'f' command (Find Metal) fails
// Sequence: 1 Arduino command + 4 movelinear + 2 gripper operations
// Returns special code to trigger full sequence retry
```

#### 2. `Arduino_controller_node.cpp`
**Location**: `spark_stage_2_ur/src/`

**Purpose**: ROS2-Arduino serial communication bridge

**Serial Port**: `/dev/ttyUSB0` (configurable via parameter)

**Baud Rate**: 115200

**Service**: `arduino/send_command` (ArduinoCommand)
```cpp
Request:
  string command  // Single character command
Response:
  bool success
  string message  // Arduino response
```

**Publisher**: `arduino/status` (String) - Arduino status messages

**Usage**:
```bash
ros2 run spark_stage_2_ur arduino_controller_node

# Test Arduino command
ros2 service call /arduino/send_command spark_stage_2_ur/srv/ArduinoCommand "{command: 'h'}"
```

#### 3. Connector Position Manager
**Location**: `spark_stage_2_ur/include/spark_stage_2_ur/connector_position.hpp`

**Purpose**: Manage connector hole positions with calibration and transform

**Grid Layout**: 2 rows × 12 columns = 24 positions (0-indexed: 0-23)

**YAML File**: `spark_stage_2_ur/connector_poses.yaml`

**Key Methods**:
```cpp
geometry_msgs::msg::Pose getConnectorPose(int row, int col);
// Returns pose for connector at (row, col)
// Uses affine transform for interpolation

void saveConnectorPose(int row, int col, const Pose& pose);
// Save calibrated pose to YAML

void loadFromYAML(const string& filename);
// Load all poses from YAML file
```

**Affine Transform Model**:
```cpp
// Position calculated using least-squares fit:
x = a + b*col + c*row
y = d + e*col + f*row

// Coefficients (from calibration):
a = 0.236333, b = 0.002000, c = 0.002500
d = -0.667833, e = 0.001667, f = -0.002750
```

---

### Arduino Code (Stage 2)

**Location**: `spark_stage_2_ur/arduino/finalwithresponse/sketch/`

**Purpose**: Stepper motor control for wire jig positioning and metal detection

**Key Functions**:
- Wire advancement into jig
- Metal ferrule detection
- Stepper homing sequences
- Real-time position feedback

**Configuration** (in Arduino sketch):
```cpp
// Hardcoded parameters at top of sketch:
#define MOTOR0_STEPS_PER_REV  200
#define MOTOR1_STEPS_PER_REV  200
#define MOTOR0_HOME_PIN       2
#define MOTOR1_HOME_PIN       3
#define METAL_DETECT_PIN      A0
```

**⚠️ IMPORTANT**: Close Arduino Serial Monitor before launching ROS nodes (port will be busy)

**Upload Process**:
1. Open Arduino IDE
2. Load sketch from `spark_stage_2_ur/arduino/finalwithresponse/sketch/`
3. Select board: Arduino Mega 2560
4. Select port: `/dev/ttyUSB0` (or appropriate)
5. Upload code
6. **CLOSE Serial Monitor** before running ROS nodes

---

### GUI System

#### Backend (FastAPI)
**Location**: `spark_minda_gui_backend/`

**Port**: 8000

**Key Routes**:
```python
/video/ws          - WebSocket for video streaming
/set_grid_positions - Service to execute connector insertion
/process_status    - Subscribe to process status updates
/wire_quality      - Subscribe to wire quality results
/grid_position     - Get current grid position
/preview_image     - Get static image snapshot
```

**ROS2 Nodes in Backend**:
```python
video_node          - VideoSubscriber (camera feed)
process_status_node - ProcessStatusSubscriber
grid_position_node  - GridPositionSubscriber
damaged_wire_node   - DamagedWireSubscriber (wire quality)
```

**Camera Topic**: `/camera/camera/color/image_raw`
- RealSense D435 RGB stream
- Resolution: 1920x1080 @ 30 FPS
- Encoding: rgb8

#### Frontend (React)
**Location**: `spark_minda_gui_frontend/`

**Port**: 5173 (Vite dev server)

**Features**:
- Real-time video stream display
- Grid position selection (2x12 grid)
- Process status monitoring
- Wire quality indicator
- Manual connector position triggering

---

## Hardware Setup

### UR10e Robot
- **IP Address**: 10.42.0.86 (default)
- **Connection**: Ethernet direct connection
- **Controller**: URCaps with Scaled Joint Trajectory Controller

### Grippers

#### Pneumatic Grippers (ngripper)
- **Gripper 0** (Wire pickup):
  - DO4 (HIGH) = Close
  - DO5 (HIGH) = Open
  - Both LOW = Neutral

- **Gripper 1** (Jig hold):
  - DO1 (HIGH) = Close
  - DO2 (HIGH) = Open

- **Gripper 2** (Tip hold):
  - DO6 (HIGH) = Close
  - DO7 (HIGH) = Open

#### Electric Gripper (egripper/egripper2)
- **Pin 16**: Large opening (not typically used)
- **Pin 17**: Small opening (for tip gripping)
- **Both LOW**: Fully closed

### Sensors

#### Vacuum Sensor
- **Topic**: `/io_and_status_controller/io_states`
- **Type**: Analog input (pressure sensor)
- **Detection**: Voltage drop indicates wire presence
- **Threshold**: ~0.035V drop
- **Baseline**: Calibrated during first 3 readings

#### Force/Torque Sensor
- **Topic**: `/force_torque_sensor_broadcaster/wrench`
- **Type**: 6-axis F/T sensor
- **Detection**: High force (>50N) indicates collision/contact
- **Use**: Emergency stop and contact detection

### Intel RealSense D435
- **USB Connection**: Direct to control PC
- **Supported Streams**:
  - RGB: 1920x1080 @ 30 FPS
  - Depth: Aligned to color frame
- **Launch Parameters**: See [System Bringup](#system-bringup)

---

## Software Setup

### Prerequisites

1. **ROS2 Humble** (Ubuntu 22.04)
```bash
sudo apt install ros-humble-desktop-full
```

2. **MoveIt 2**
```bash
sudo apt install ros-humble-moveit
```

3. **UR ROS2 Drivers**
```bash
sudo apt install ros-humble-ur-robot-driver
sudo apt install ros-humble-ur-moveit-config
```

4. **RealSense SDK**
```bash
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description
```

5. **Python Dependencies** (for GUI backend)
```bash
cd spark_minda_gui_backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

6. **Node.js & npm** (for GUI frontend)
```bash
cd spark_minda_gui_frontend
npm install
```

### Building the Workspace

```bash
cd ~/spark_ws
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Build specific package
colcon build --packages-select spark_stage_2_ur

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the workspace
source install/setup.bash
```

---

## System Bringup

### Complete System Startup (Stage 2)

**Terminal 1: UR10e Robot Driver**
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10e \
    robot_ip:=10.42.0.86
```

**Terminal 2: MoveIt Motion Planning**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur10e \
    launch_rviz:=true
```

**Terminal 3: Main Control Node + Arduino**
```bash
# Upload Arduino code FIRST (and close Serial Monitor)

# Launch main control node
ros2 launch spark_stage_2_ur bringup_moveit_and_node.launch.py

# In separate terminal, launch Arduino controller
ros2 run spark_stage_2_ur arduino_controller_node
```

**Terminal 4: RealSense Camera**
```bash
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=1920,1080,30 \
    align_depth.enable:=true
```

**Terminal 5: GUI Backend**
```bash
cd ~/spark_ws/src/spark_minda_gui_backend
source venv/bin/activate
source run_backend.sh
# Or manually:
# uvicorn app.main:app --host 0.0.0.0 --port 8000
```

**Terminal 6: GUI Frontend**
```bash
cd ~/spark_ws/src/spark_minda_gui_frontend
source run_frontend.sh
# Or manually:
# npm run dev
```

**Access GUI**: Open browser to `http://localhost:5173`

---

### Quick Test (Without GUI)

```bash
# Test single connector insertion at row=2, col=5
ros2 service call /move_to_pose std_srvs/srv/Trigger
```

---

### Connector Calibration GUI

To calibrate and save connector hole positions:

```bash
ros2 launch spark_stage_2_ur connector_pose_gui.launch.py
```

**Calibration Process**:
1. Move robot to each connector hole manually
2. Set row/column in GUI
3. Click "Save Position"
4. Repeat for representative holes (recommend calibrating at least 4 corners + center)
5. Positions saved to `connector_poses.yaml`
6. Other positions calculated via affine transform

---

## Programming Guide

### Adding New Sequence Steps

1. **Open** `spark_stage_2_ur/src/move_to_pose_node.cpp`

2. **Find** `run_connector_sequence()` function

3. **Add step** using the retry wrapper:

```cpp
// Step X: Your new step
if (!execute_with_retry("Step X: Description", [&]() {
    // Your operations here
    auto [ok, msg] = movelinear(x, y, z, qx, qy, qz, qw);
    if (!ok) return false;
    
    if (!ngripper(true, 0)) return false;
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Must return true on success
    return true;
})) {
    return {false, "Step X: Description failed"};
}
```

4. **Rebuild**:
```bash
colcon build --packages-select spark_stage_2_ur
source install/setup.bash
```

### Creating New Arduino Commands

1. **Add command** to Arduino sketch:
```cpp
// In Arduino code
case 'x':  // Your new command
    // Your Arduino logic
    Serial.println("X_COMPLETE");
    break;
```

2. **Add to ROS node** (`Arduino_controller_node.cpp`):
```cpp
case 'x':
case 'X':
    command_name = "Your Command Name";
    command = 'x';
    break;
```

3. **Use in sequence**:
```cpp
if (!send_arduino_command('x')) return false;
```

### Modifying Connector Grid Size

1. **Update** `set_grid_positions_callback()` in `move_to_pose_node.cpp`:
```cpp
const int cols = 12;  // Change column count
const int rows = 2;   // Change row count
```

2. **Update** grid position calculation:
```cpp
int position = (row - 1) * cols + (col - 1);
```

3. **Update GUI** grid component to match

---

## Services Reference

### Stage 2 Services

```bash
# Execute connector sequence (row=2, col=5 hardcoded)
ros2 service call /move_to_pose std_srvs/srv/Trigger

# Execute multiple grid positions
ros2 service call /set_grid_positions \
    pcb_visualization_msgs/srv/SetGridPositions \
    "{positions: [0, 1, 2, 5, 10, 15]}"

# Linear Cartesian move with quaternion
ros2 service call /move_linear_quat \
    spark_stage_2_ur/srv/MoveLinearQuat \
    "{trans_x: 0.5, trans_y: -0.3, trans_z: 0.4, 
      quat_x: 0.0, quat_y: 0.0, quat_z: 0.0, quat_w: 1.0}"

# Move to joint angles (radians)
ros2 service call /move_to_joints \
    spark_stage_2_ur/srv/MoveToJoints \
    "{joint_positions: [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]}"

# Send Arduino command
ros2 service call /arduino/send_command \
    spark_stage_2_ur/srv/ArduinoCommand \
    "{command: 'h'}"
```

### Stage 1 Services

```bash
# Execute main pick-place sequence
ros2 service call /execute_main_sequence std_srvs/srv/Trigger

# Wire tip detection sequence 1
ros2 service call /execute_seq1 std_srvs/srv/Trigger

# Wire tip detection sequence 2
ros2 service call /execute_seq2 std_srvs/srv/Trigger

# Toggle relay
ros2 service call /toggle_relay std_srvs/srv/Trigger

# Get tip X coordinate (test node)
ros2 service call /get_tip_x_coord std_srvs/srv/Trigger
```

---

## Topics Reference

### Published Topics

```bash
# Process status updates
/process_status                          # String: Current operation status

# Grid position tracking
/grid_position_update                    # Int32: Current grid position (0-23)

# Wire quality indicator
/wire_quality                            # Bool: true=pass, false=fail

# Arduino status
/arduino/status                          # String: Arduino responses

# Vision (Stage 1)
/vision_x_coordinate                     # Float32: Wire tip X position

# Camera feed
/camera/camera/color/image_raw           # Image: RGB stream
/camera/camera/depth/image_rect_raw      # Image: Depth stream
```

### Subscribed Topics

```bash
# Force/torque sensor
/force_torque_sensor_broadcaster/wrench  # WrenchStamped

# Digital/analog inputs
/io_and_status_controller/io_states      # IOStates

# Joint states
/joint_states                            # JointState
```

---

## Troubleshooting

### Common Issues

#### 1. **"Serial port busy" error**
**Cause**: Arduino Serial Monitor is open
**Solution**: Close Arduino IDE's Serial Monitor before running ROS nodes

#### 2. **Robot not moving**
**Checks**:
- UR10e is powered on and in remote control mode
- Driver connected: `ros2 topic list | grep joint_states`
- MoveIt running: Check RViz display
- No E-stop activated on robot

#### 3. **Camera not detected**
```bash
# Check if RealSense is connected
rs-enumerate-devices

# Test camera
ros2 topic hz /camera/camera/color/image_raw

# Check USB connection and permissions
ls -la /dev/video*
sudo usermod -aG video $USER  # May need to logout/login
```

#### 4. **Arduino not responding**
```bash
# Check serial port
ls -la /dev/ttyUSB*

# Add user to dialout group
sudo usermod -aG dialout $USER  # Logout/login required

# Test serial connection
ros2 run spark_stage_2_ur arduino_controller_node --ros-args -p serial_port:=/dev/ttyUSB0
```

#### 5. **GUI not showing images**
**Backend Checks**:
```bash
# Verify camera topic in backend
grep "/camera/camera/color/image_raw" src/spark_minda_gui_backend/app/ros_nodes_and_routes/ros/*.py

# Check WebSocket connection
# Look for "WebSocket connected - video stream starting" in backend logs

# Verify cv_bridge is installed
pip list | grep cv-bridge
```

**Restart backend** after changing camera topics:
```bash
# Find and kill backend process
ps aux | grep uvicorn | grep -v grep
kill <PID>

# Restart
cd spark_minda_gui_backend
source venv/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

#### 6. **"Wire quality: FAIL" repeatedly**
**Cause**: Metal detection ('f' command) not detecting ferrule
**Checks**:
- Verify metal sensor connection in Arduino
- Check wire has metallic ferrule
- Adjust detection threshold in Arduino code
- Test recovery sequence triggers

#### 7. **Gripper not opening/closing**
**Pin Verification**:
```bash
# Manually test gripper pins
ros2 service call /io_and_status_controller/set_io \
    ur_msgs/srv/SetIO \
    "{fun: 1, pin: 4, state: 1.0}"

# Check IO states
ros2 topic echo /io_and_status_controller/io_states --once
```

**Common Pin Issues**:
- Wrong pin numbers in code vs hardware
- Solenoid valves not powered
- Air pressure insufficient

#### 8. **Build errors**
```bash
# Clean build
rm -rf build install log
colcon build

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild with verbose output
colcon build --event-handlers console_direct+
```

---

## Configuration Files

### Key Configuration Locations

1. **Connector Positions**:
   - File: `spark_stage_2_ur/connector_poses.yaml`
   - Format: YAML dictionary of poses per (row, col)

2. **MoveIt Configuration**:
   - Package: `ur10_e_moveit_config/`
   - Kinematics: `config/kinematics.yaml`
   - Planning: `config/ompl_planning.yaml`

3. **Launch Parameters**:
   - Main launch: `spark_stage_2_ur/launch/bringup_moveit_and_node.launch.py`
   - Velocity/acceleration scaling parameters

4. **Arduino Serial Port**:
   - Default: `/dev/ttyUSB0`
   - Change via ROS parameter or Arduino node code

5. **Robot IP**:
   - Default: `10.42.0.86`
   - Change in UR launch command

---

## Performance Tuning

### Motion Speed

Adjust velocity/acceleration in `move_to_pose_node.cpp`:

```cpp
// In constructor
velocity_scaling_ = 0.05;      // 0.01-1.0 (5% of max)
acceleration_scaling_ = 0.05;  // 0.01-1.0 (5% of max)

// For specific step (e.g., connector insertion):
saved_velocity = velocity_scaling_;
velocity_scaling_ = 0.02;  // Slower for precision
move_group.setMaxVelocityScalingFactor(velocity_scaling_);
// ... execute movement ...
velocity_scaling_ = saved_velocity;  // Restore
```

### Vision Frame Rate

Adjust camera FPS in launch:
```bash
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=1920,1080,30  # Change 30 to desired FPS
```

Lower resolution for higher FPS:
```bash
rgb_camera.color_profile:=640,480,60
```

### WebSocket Streaming

Adjust frame rate in `video.py`:
```python
await asyncio.sleep(0.033)  # ~30 FPS
# Change to 0.066 for ~15 FPS or 0.016 for ~60 FPS
```

---

## Safety Considerations

1. **Emergency Stop**: Always have robot E-stop accessible
2. **Workspace**: Keep workspace clear during operation
3. **Force Limits**: Force sensor will trigger on >50N
4. **Speed**: Start with low velocity_scaling (0.05) when testing
5. **Collision**: MoveIt checks for self-collision
6. **Manual Mode**: Test sequences in manual mode first

---

## Version History

- **v2.0** (Current): Stage 2 wire insertion system with multi-gripper EOAT
- **v1.0**: Stage 1 wire straightening with vision-guided picking

---

## Support & Contact

For technical issues or questions:
- Check troubleshooting section above
- Review code comments in key files
- Check ROS logs: `~/.ros/log/latest/`

---

## License

Proprietary - SPARK MINDA Internal Use Only
