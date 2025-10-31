# UR10e MoveIt Configuration - Standalone Package

This is a **standalone** MoveIt2 configuration package for the UR10e robot. Unlike the standard `ur_moveit_config` package, this package:

1. **Contains all necessary URDF files** - No dependency on `ur_description` package
2. **Uses a static SRDF file** - The SRDF is loaded directly without xacro processing on every launch
3. **Includes all mesh files** - All visual and collision meshes are included in the package
4. **Self-contained configuration** - All kinematics, joint limits, and physical parameters are included

## Why Standalone?

This package was created to address loading and description issues with the default UR driver and packages. By making the package standalone:

- Faster launch times (no URDF/SRDF generation on each launch)
- More reliable robot description loading
- Easier to customize and maintain
- No external package dependencies for robot description

## Package Structure

```
ur10_e_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── ur10e/
│   │   ├── default_kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── physical_parameters.yaml
│   │   └── visual_parameters.yaml
│   ├── initial_positions.yaml
│   ├── controllers.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── ompl_planning.yaml
│   └── ur_servo.yaml
├── launch/
│   └── move_group.launch.py  # Main launch file
├── meshes/
│   └── ur10e/
│       ├── collision/
│       └── visual/
├── rviz/
│   └── view_robot.rviz
├── srdf/
│   └── ur10e.srdf  # Static SRDF (not generated)
└── urdf/
    ├── ur10e.urdf.xacro  # UR10e-specific URDF
    ├── ur_macro.xacro
    ├── ur.ros2_control.xacro
    └── inc/  # Include files

```

## Usage

### Launch MoveIt with Move Group

```bash
ros2 launch ur10_e_moveit_config move_group.launch.py
```

### Launch with specific robot IP

```bash
ros2 launch ur10_e_moveit_config move_group.launch.py robot_ip:=192.168.1.100
```

### Launch without RViz

```bash
ros2 launch ur10_e_moveit_config move_group.launch.py launch_rviz:=false
```

### Launch with MoveIt Servo

```bash
ros2 launch ur10_e_moveit_config move_group.launch.py launch_servo:=true
```

## Key Differences from ur_moveit_config

1. **Static SRDF Loading**: The SRDF file (`srdf/ur10e.srdf`) is loaded directly as a file, not processed through xacro
2. **Embedded URDF**: All URDF files are contained within this package
3. **No External Dependencies**: Doesn't depend on `ur_description` package
4. **UR10e-specific**: Configured specifically for UR10e (not a generic UR package)

## Building

```bash
cd ~/spark_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select ur10_e_moveit_config
source install/setup.bash
```

## Customization

### Modifying Robot Kinematics

Edit `config/ur10e/default_kinematics.yaml` to adjust kinematic parameters specific to your robot.

### Modifying SRDF

The SRDF file is located at `srdf/ur10e.srdf`. You can edit this file directly to:
- Add/modify planning groups
- Add/modify group states (poses)
- Modify collision checking pairs

After editing, rebuild the package for changes to take effect.

### Modifying Visual Appearance

Edit `config/ur10e/visual_parameters.yaml` to change mesh files or materials.

## Troubleshooting

If you encounter issues with the robot description:

1. Verify all mesh files are present in `meshes/ur10e/`
2. Check that the SRDF file matches your URDF link/joint names
3. Ensure the package is properly built and sourced

## License

Apache 2.0 (same as original UR packages)
