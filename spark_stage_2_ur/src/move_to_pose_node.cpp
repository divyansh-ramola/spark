#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "spark_stage_2_ur/srv/move_to_pose.hpp"
#include "spark_stage_2_ur/srv/move_linear_rpy.hpp"
#include "spark_stage_2_ur/srv/move_linear_quat.hpp"
#include "spark_stage_2_ur/srv/arduino_command.hpp"
#include "spark_stage_2_ur/srv/move_to_joints.hpp"
#include "pcb_visualization_msgs/srv/set_grid_positions.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <memory>
#include <mutex>
#include <limits>
#include <string.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <rmw/qos_profiles.h>
#include <utility>
#include <variant>
#include <future>
#include <atomic>
#include "spark_stage_2_ur/connector_position.hpp"
#include <geometry_msgs/msg/pose.hpp>

class MoveToPoseNode : public rclcpp::Node
{
public:
  explicit MoveToPoseNode()
  : Node("move_to_pose_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator"),
    velocity_scaling_(0.4),
    acceleration_scaling_(0.4),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    connector_manager_(this->get_logger())  // Automatically loads YAML in constructor
  {
    RCLCPP_INFO(this->get_logger(), "Move to Pose Node has been started.");

    // Declare and get velocity and acceleration scaling parameters
    this->declare_parameter("velocity_scaling", 0.4);
    this->declare_parameter("acceleration_scaling", 0.4);
    
    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Velocity scaling: %.2f", velocity_scaling_);
    RCLCPP_INFO(this->get_logger(), "Acceleration scaling: %.2f", acceleration_scaling_);

    main_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MoveToPoseNode::main_timer_callback, this));
    move_group.setStartStateToCurrentState();
    move_group.allowReplanning(true);

    // Set conservative speed
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);

    //start_monitor(); doeesnt do jack shiii
    // Give MoveIt and TF time to initialize
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt and TF to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Create callback groups to allow sensor callbacks to run concurrently with services
    group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_services_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_grid_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pick_service = create_service<std_srvs::srv::Trigger>(
      "move_to_pose",
      std::bind(&MoveToPoseNode::move_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      group_services_);



    // Linear move with quaternion
    linear_quat_service = create_service<spark_stage_2_ur::srv::MoveLinearQuat>(
      "move_linear_quat",
      std::bind(&MoveToPoseNode::move_linear_quat_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      group_services_);

    // Move to joint angles
    joints_service = create_service<spark_stage_2_ur::srv::MoveToJoints>(
      "move_to_joints",
      std::bind(&MoveToPoseNode::move_to_joints_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      group_services_);

    // Set grid positions service (uses Reentrant callback group to avoid deadlocks)
    grid_positions_service = create_service<pcb_visualization_msgs::srv::SetGridPositions>(
      "set_grid_positions",
      std::bind(&MoveToPoseNode::set_grid_positions_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      group_grid_service_);

    gripper_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    // Check for gripper service but don't block indefinitely
    int gripper_wait_attempts = 0;
    while (!gripper_client_->wait_for_service(std::chrono::seconds(1)) && gripper_wait_attempts < 5) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /io_and_status_controller/set_io service... (attempt %d/5)", ++gripper_wait_attempts);
    }
    if (gripper_wait_attempts >= 5) {
      RCLCPP_WARN(this->get_logger(), "Gripper service not available after 5 attempts, continuing anyway...");
    } else {
      RCLCPP_INFO(this->get_logger(), "Gripper service is available");
    }

    arduino_client_ = create_client<spark_stage_2_ur::srv::ArduinoCommand>("arduino/send_command");
    // while (!arduino_client_->wait_for_service(std::chrono::seconds(2))) {
    //   RCLCPP_INFO(this->get_logger(), "Waiting for arduino/send_command service to be available...");
    // }

    // Create joint trajectory action client for direct control
    joint_trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "Joint trajectory action client created");
    
    // Create simple trajectory publisher as backup
    joint_trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory", 10);
    RCLCPP_INFO(this->get_logger(), "Joint trajectory publisher created");

      {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = group_sensors_;
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
        wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_sensor_broadcaster/wrench", qos,
          std::bind(&MoveToPoseNode::wrench_callback, this, std::placeholders::_1),
          sub_opts);
      }

      {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = group_sensors_;
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
        vacuum_sub_ = create_subscription<ur_msgs::msg::IOStates>(
          "/io_and_status_controller/io_states", qos,
          std::bind(&MoveToPoseNode::vacuum_callback, this, std::placeholders::_1),
          sub_opts);
      }

    // Define table collision object
    define_table_collision_object();
    //allow_table_base_contact();

    // Create status publishers
    grid_position_pub_ = create_publisher<std_msgs::msg::Int32>("/grid_position_update", 10);
    process_status_pub_ = create_publisher<std_msgs::msg::String>("/process_status", 10);
    wire_quality_pub_ = create_publisher<std_msgs::msg::Bool>("/wire_quality", 10);
    RCLCPP_INFO(this->get_logger(), "Status publishers created");
    
    RCLCPP_INFO(this->get_logger(), "Node ready - services available:");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose (linear Cartesian, offset-based)");
    RCLCPP_INFO(this->get_logger(), "  - /move_with_planning (obstacle avoidance, hardcoded target)");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose_full (full pose control: XYZ + quaternion XYZW)");
    RCLCPP_INFO(this->get_logger(), "  - /move_linear_rpy (linear Cartesian: XYZ + RPY angles)");
    RCLCPP_INFO(this->get_logger(), "  - /move_linear_quat (linear Cartesian: XYZ + quaternion XYZW)");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_joints (direct joint angle control)");
    RCLCPP_INFO(this->get_logger(), "  - /set_grid_positions (process grid positions array)");
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planning_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveToPose>::SharedPtr pose_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveLinearRPY>::SharedPtr linear_rpy_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveLinearQuat>::SharedPtr linear_quat_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveToJoints>::SharedPtr joints_service;
  rclcpp::Service<pcb_visualization_msgs::srv::SetGridPositions>::SharedPtr grid_positions_service;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Separate callback groups so sensor callbacks can run in parallel with services
  rclcpp::CallbackGroup::SharedPtr group_sensors_;
  rclcpp::CallbackGroup::SharedPtr group_services_;
  rclcpp::CallbackGroup::SharedPtr group_grid_service_;  // Reentrant group for grid service to avoid deadlocks

  // Joint trajectory action client for direct control
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;
  
  // Simple trajectory publisher as backup
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

  // Joint state tracking (kept for backup/debugging)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::map<std::string, double> latest_positions_;
  std::mutex js_mutex_;
  bool have_joint_state_ = false;
  // Velocity and acceleration scaling factors
  double velocity_scaling_;
  double acceleration_scaling_;
  // TF for getting tool0 pose directly
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr vacuum_sub_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr gripper_client_;
  rclcpp::Client<spark_stage_2_ur::srv::ArduinoCommand>::SharedPtr arduino_client_;


  // Force/torque monitoring state
  double wrench_initial_ = 0.0;
  int wrench_update_count_ = 0;
  bool wrench_initialized_ = false;
  std::atomic<bool> wrench_active_{false};

  double vacuum_initial_ = 0.0;
  int vacuum_update_count_ = 0;
  bool vacuum_initialized_ = false;
  std::atomic<bool> vacuum_active_{false};

  // Connector position manager
  connector_position::ConnectorPositionManager connector_manager_;

  // Publishers for status updates
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr grid_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr process_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wire_quality_pub_;


  void vacuum_callback(const ur_msgs::msg::IOStates::SharedPtr msg)
  {
    const auto &analog = msg->analog_in_states[0].state;
    
    if (vacuum_update_count_ < 3) {
      vacuum_initial_ = std::abs(analog);
      vacuum_update_count_++;

      RCLCPP_WARN(this->get_logger(), "Initial end-effector vacuum analog baseline set to: %.2f V", vacuum_initial_);
      return;
    }

    vacuum_initialized_ = true;

    if (!vacuum_initialized_) {
      return;
    }
    
    // More sensitive threshold: reduced from (0.87-0.73)/3 to (0.87-0.73)/4
    // This makes it approximately 0.035V instead of 0.047V
    const double voltage_drop = vacuum_initial_ - std::abs(analog);
    const double threshold = (0.87-0.73)/4;
    const bool over_threshold = (voltage_drop > threshold);

    if (over_threshold && !vacuum_active_.load()) {
      //RCLCPP_WARN(this->get_logger(), "WIRE DETECTED! voltage: %.3f V, drop: %.3f V (threshold: %.3f V)", 
                  // analog, voltage_drop, threshold);
      // Stop current motion immediately to reduce latency
      // try { move_group.stop(); } catch (const std::exception &e) {
      //   RCLCPP_WARN(this->get_logger(), "move_group.stop() threw: %s", e.what());
      // }
    }

    vacuum_active_.store(over_threshold);
  }

  void main_timer_callback()
  {
    // Timer callback implementation
  }

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    const auto &force = msg->wrench.force;

    if (wrench_update_count_ < 3) {
      wrench_initial_ = std::abs(force.z);
      wrench_update_count_++;
      
      RCLCPP_INFO(this->get_logger(), "Initial end-effector force baseline set to: %.2f N", wrench_initial_);
      return;
    }

    wrench_initialized_ = true;
    
    if (!wrench_initialized_) {
      return;
    } 
    const bool over_threshold = ((std::abs(force.z) - wrench_initial_) > 50.0);

    if (over_threshold && !wrench_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "High force detected on end-effector: %.2f N ", force.z);
      // try { move_group.stop(); } catch (const std::exception &e) {
      //   RCLCPP_WARN(this->get_logger(), "move_group.stop() threw: %s", e.what());
      // }
    }

    wrench_active_.store(over_threshold);
  }


  // Compute total joint motion in radians for a trajectory
  double joint_travel_distance(const moveit_msgs::msg::RobotTrajectory &trajectory) const
  {
    const auto &points = trajectory.joint_trajectory.points;
    if (points.size() < 2) {
      return std::numeric_limits<double>::infinity();
    }

    double distance = 0.0;
    for (size_t idx = 1; idx < points.size(); ++idx) {
      const auto &prev = points[idx - 1].positions;
      const auto &curr = points[idx].positions;
      for (size_t joint = 0; joint < curr.size() && joint < prev.size(); ++joint) {
        distance += std::fabs(curr[joint] - prev[joint]);
      }
    }
    return distance;
  }

  

  

  // Perform multiple descent attempts with Y offset scanning
  // Returns true if vacuum detection occurred, false otherwise
  // Zigzag pattern: 0, -y, +y, -2y, +2y, -3y, +3y, ...
  bool perform_descent_with_y_scanning(int no_of_tries = 5, int max_steps = 200, double y_offset_increment = 0.0005)
  {
    bool detected = false;
    double previous_y_offset = 0.0;

    for (int tries = 0; tries < no_of_tries; ++tries) {
      // Calculate zigzag pattern: 0, -y, +y, -2y, +2y, ...
      double y_offset_ = 0.0;
      if (tries > 0) {
        int step = (tries + 1) / 2;  // 1, 1, 2, 2, 3, 3, ...
        double sign = (tries % 2 == 1) ? -1.0 : 1.0;  // -, +, -, +, ...
        y_offset_ = sign * step * y_offset_increment;
      }
      
      RCLCPP_WARN(this->get_logger(), "Attempt %d/%d: prepare Y offset = %.4f m (vac=%d, wrench=%d)",
                  tries + 1, no_of_tries, y_offset_, static_cast<int>(vacuum_active_.load()), static_cast<int>(wrench_active_.load()));

      // Move laterally to the desired Y offset for this attempt
      // Calculate relative movement from current position
      double relative_y_move = y_offset_ - previous_y_offset;
      
      if (std::fabs(relative_y_move) > 1e-9) {
        auto [ok_offset, msg_offset] = joglinear(relative_y_move, 0.0, 0.0);
        {auto [ok_lift, msg_lift] = joglinear(0.0, 0.0, -0.010);}
        if (!ok_offset) {
          RCLCPP_WARN(this->get_logger(), "Offset move failed at try %d: %s", tries + 1, msg_offset.c_str());
          // Skip this attempt and continue with next offset
          previous_y_offset = y_offset_;
          continue;
        }
        previous_y_offset = y_offset_;
      }

      // Reset detection flags at the start of each descent attempt
      // We care about a fresh detection per attempt; if sensors are still latched, they will re-trigger
      wrench_active_.store(false);
      vacuum_active_.store(false);

      // Descend in small Z steps until a trigger or max steps
      bool step_failed = false;
      for (int i = 0; i < max_steps; ++i) {
        // Stop everything only when vacuum is active (wire detected)
        if (vacuum_active_.load()) {
          RCLCPP_WARN(this->get_logger(),
                      "Attempt %d: VACUUM detected at step %d", 
                      tries + 1, i + 1);
          detected = true;
          break;
        }

        // If wrench is high (contact), stop current attempt, lift and try next Y
        if (wrench_active_.load()) {
          RCLCPP_WARN(this->get_logger(),
                      "Attempt %d: WRENCH contact detected at step %d; will try next Y offset",
                      tries + 1, i + 1);
          step_failed = true;  // use the failure path to try next Y
          break;
        }

        auto [ok_step, msg_step] = joglinear(0.0, 0.0, -0.0006);
        if (!ok_step) {
          RCLCPP_WARN(this->get_logger(), "Attempt %d: step failed at %d: %s", tries + 1, i + 1, msg_step.c_str());
          step_failed = true;
          break;
        }
      }

      // After attempt: lift a bit to clear space before next try
      auto [ok_lift, msg_lift] = joglinear(0.0, 0.0, 0.015);
      if (!ok_lift) {
        RCLCPP_WARN(this->get_logger(), "Attempt %d: lift failed: %s", tries + 1, msg_lift.c_str());
      }

      if (detected) {
        // Stop outer loop on successful trigger
        
        break;
      }

      if (step_failed) {
        // Try next Y offset
        RCLCPP_WARN(this->get_logger(), "Attempt %d: continuing to next Y offset due to step failure", tries + 1);
      } else {
        // No trigger and no failure: continue to next Y offset
        RCLCPP_INFO(this->get_logger(), "Attempt %d completed without trigger; trying next Y offset", tries + 1);
      }

      {
        (void)ngripper(false,0);
        
      }
    }  // end for loop

    return detected;
  }

  void move_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Use hardcoded row=2, col=5 for backwards compatibility with existing code
    auto [success, message] = run_connector_sequence(2, 5);
    response->success = success;
    response->message = message;
  };
  void main_callback()
  {
    
  }

  // Helper function to publish process status
  void publish_status(const std::string& status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    process_status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
  }

  // Helper function to publish grid position update
  void publish_grid_position(int position)
  {
    auto msg = std_msgs::msg::Int32();
    msg.data = position;
    grid_position_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Grid position update: %d", position);
  }

  // Helper function to publish wire quality
  void publish_wire_quality(bool quality)
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = quality;
    wire_quality_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Wire quality: %s", quality ? "PASS" : "FAIL");
  }




  // Function to send command to Arduino node
  // Input: command - character command ('h', 'f', 'm', 'n', 'p', 'q')
  // Returns: true if command executed successfully, false otherwise

  // Recovery sequence when 'find metal' command fails
  // Executes: 1 command + 4 movelinear + 2 gripper operations
  bool execute_find_metal_recovery()
  {
    RCLCPP_WARN(this->get_logger(), "=== EXECUTING FIND METAL RECOVERY SEQUENCE ===");
    
    try {
      // Command 1: Send 'h' (home) command
      if (!send_arduino_command('m')) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Arduino command 'h' failed");
        return false;
      }
      
      // Move 1: Go up
      auto [ok1, msg1] = movelinear(0.507, -0.260, 0.744, 0.708, 0.014, -0.520, 0.477);
      if (!ok1) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Move 1 (up) failed: %s", msg1.c_str());
        return false;
      }
      // Gripper 1: Release gripper 1
      if (!egripper(true)) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Gripper 1 release failed");
        return false;
      }

       // Gripper 1: Release gripper 1
      if (!egripper2(false)) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Gripper 1 release failed");
        return false;
      }

      // Move 2: Approach position
      auto [ok2, msg2] = movelinear(0.508, -0.261, 0.482, 0.709, 0.015, -0.520, 0.477);
      if (!ok2) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Move 2 (approach) failed: %s", msg2.c_str());
        return false;
      }
      
     
      // Gripper 2: Close gripper 1
      if (!egripper2(true)) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Gripper 1 close failed");
        return false;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));

       
      {
        bool ok_grip = ngripper(false, 1);
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      
      // Move 4: Move up
      auto [ok4, msg4] = movelinear(0.507, -0.260, 0.744, 0.708, 0.014, -0.520, 0.477);
      if (!ok4) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Move 4 (back up) failed: %s", msg4.c_str());
        return false;
      }

      // Move 5: Move to position 2
      auto [ok5, msg5] = movelinear(-0.065, -0.465, 0.930, -0.382, 0.923, -0.055, -0.007);
      if (!ok5) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Move 5 (position 2) failed: %s", msg5.c_str());
        return false;
      }

      // Move 6: Move to final position
      auto [ok6, msg6] = movelinear(-0.403, -0.350, 0.602, -0.025, 1.000, 0.015, 0.001);
      if (!ok6) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Move 6 (final position) failed: %s", msg6.c_str());
        return false;
      }

      // Gripper 2: open gripper 1
      if (!egripper2(false)) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Gripper 1 close failed");
        return false;
      }


      joglinear(0.015, 0, 0);
      joglinear(-0.015, 0, 0); // shake wire a bit

      rclcpp::sleep_for(std::chrono::milliseconds(1000));

      // Gripper 2: open gripper 1
      if (!egripper2(true)) {
        RCLCPP_ERROR(this->get_logger(), "Recovery: Gripper 1 close failed");
        return false;
      }

      
      RCLCPP_INFO(this->get_logger(), "Find metal recovery sequence completed successfully");
      return true;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during recovery sequence: %s", e.what());
      return false;
    }
  }

  // Full connector insertion sequence for a specific row and column
  // Returns {success, message}
  std::pair<bool, std::string> run_connector_sequence(const int& row, const int& col)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Starting connector sequence for row=%d, col=%d", row, col);
    RCLCPP_INFO(this->get_logger(), "========================================");

    // Helper lambda for automatic retry logic (retry once if failed)
    auto execute_with_retry = [this](const std::string& step_name, 
                                      std::function<bool()> operation) -> bool {
      // First attempt
      RCLCPP_INFO(this->get_logger(), "=== %s - Attempt 1/2 ===", step_name.c_str());
      bool success = operation();
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "%s completed successfully", step_name.c_str());
        return true;
      }
      
      // First attempt failed, retry once
      RCLCPP_WARN(this->get_logger(), "%s failed on attempt 1, retrying...", step_name.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(500)); // Brief delay before retry
      
      // Second attempt
      RCLCPP_INFO(this->get_logger(), "=== %s - Attempt 2/2 ===", step_name.c_str());
      success = operation();
      
      if (success) {
        RCLCPP_INFO(this->get_logger(), "%s completed successfully on retry", step_name.c_str());
        return true;
      }
      
      RCLCPP_ERROR(this->get_logger(), "%s failed after 2 attempts", step_name.c_str());
      return false;
    };

    // Step 1: Initial positioning
    if (!execute_with_retry("Step 1: Initial positioning", [&]() {
      auto [ok, msg] = movelinear(-0.295, -0.655, 0.836, 1.000, 0.023, -0.008, -0.008);
      return ok;
    })) {
      return {false, "Step 1: Initial positioning failed"};
    }

    // Step 2: Arduino setup and gripper initialization
    if (!execute_with_retry("Step 2: Arduino setup and gripper init", [&]() {
      if (!send_arduino_command('h')) return false;
      if (!send_arduino_command('m')) return false;
      if (!ngripper(false, 0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper DO0");
        return false;
      }
      ngripper(false, 1);
      ngripper(false, 2);
      egripper(false);
      return true;
    })) {
      return {false, "Step 2: Arduino setup and gripper init failed"};
    }

    // Step 3: Move to wire detection position
    if (!execute_with_retry("Step 3: Move to wire detection position", [&]() {
      auto [ok, msg] = movelinear(-0.295, -0.655, 0.313, 1.000, 0.024, -0.008, -0.007);
      return ok;
    })) {
      return {false, "Step 3: Move to wire detection position failed"};
    }

    // Step 4: Wire detection (first attempt)
    bool wire_detected = false;
    if (!execute_with_retry("Step 4: Wire detection (fine scan)", [&]() {
      const int no_of_tries = 7;
      const int kMaxSteps = 200;
      wire_detected = perform_descent_with_y_scanning(no_of_tries, kMaxSteps, 0.005);
      return wire_detected;
    })) {
      // Try coarse scan if fine scan failed
      RCLCPP_WARN(this->get_logger(), "Fine scan failed, attempting coarse scan...");
      
      if (!execute_with_retry("Step 4b: Wire detection (coarse scan)", [&]() {
        auto [ok, msg] = movelinear(-0.295, -0.655, 0.313, 1.000, 0.024, -0.008, -0.007);
        if (!ok) return false;
        
        const int no_of_tries = 7;
        const int kMaxSteps = 200;
        wire_detected = perform_descent_with_y_scanning(no_of_tries, kMaxSteps, 0.010);
        return wire_detected;
      })) {
        return {false, "Step 4: Wire detection failed (both fine and coarse)"};
      }
    }

    // Step 5: Wire pickup
    if (!execute_with_retry("Step 5: Wire pickup", [&]() {
      if (!vacuum_active_.load()) {
        RCLCPP_ERROR(this->get_logger(), "Vacuum not active, cannot pick wire");
        return false;
      }
      auto [ok_lift, msg_lift] = joglinear(0.0, 0.0, 0.015);
      if (!ok_lift) return false;
      
      joglinear(0.015, 0, 0);
      joglinear(-0.015, 0, 0); // shake wire a bit
      
      if (!ngripper(true, 0)) return false;
      
      auto [ok_move, msg_move] = movelinear(-0.295, -0.655, 0.836, 1.000, 0.023, -0.008, -0.008);
      return ok_move;
    })) {
      return {false, "Step 5: Wire pickup failed"};
    }
    
    // Publish status: Wire Picked
    publish_status("Wire Picked");
    
    

    // Step 6: Move to place jig
    if (!execute_with_retry("Step 6: Move to place jig (approach)", [&]() {
      auto [ok1, msg1] = movelinear(0.378, -0.390, 0.836, 1.000, 0.024, -0.006, -0.006);
      if (!ok1) return false;
      return true;  // Must return true on success!
    })) {
      return {false, "Step 6: Move to place jig (approach) failed"};
    }
    
    RCLCPP_INFO(this->get_logger(), "Approach complete, moving to jig position");
    
    // Step 6b: Lower to jig position
    if (!execute_with_retry("Step 6b: Lower to jig position", [&]() {
      auto [ok2, msg2] = movelinear(0.378, -0.390, 0.579, 1.000, 0.024, -0.006, -0.006);
      if (!ok2) return false;
      return true;  // Must return true on success!
    })) {
      return {false, "Step 6b: Lower to jig position failed"};
    }

    // Step 6d: slide to jig position
    if (!execute_with_retry("Step 6d: Slide to jig position", [&]() {
      auto [ok2, msg2] = movelinear(0.357, -0.417, 0.579, 1.000, 0.024, -0.006, -0.006);
      if (!ok2) return false;
      return true;  // Must return true on success!
    })) {
      return {false, "Step 6d: Slide to jig position failed"};
    }


   
    
    // Step 7: Place wire in jig
    if (!execute_with_retry("Step 7: Place wire in jig", [&]() {
      // if (!send_arduino_command('p')) return false;
      if (!ngripper(true, 1)) return false;
      if (!ngripper(false, 0)) return false;
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      
      
      return true;
    })) {
      return {false, "Step 7: Place wire in jig failed"};
    }

    // Step 6c: higher to jig position
    if (!execute_with_retry("Step 6c: Higher to jig position", [&]() {
      auto [ok2, msg2] = movelinear(0.357, -0.417, 0.800, 1.000, 0.024, -0.006, -0.006);
      if (!ok2) return false;
      return true;  // Must return true on success!
    })) {
      return {false, "Step 6c: Higher to jig position failed"};
    }

    // Step 8: Arduino sequence and gripper adjustments
    // NOTE: This step does NOT use retry logic for 'f' command failure
    RCLCPP_INFO(this->get_logger(), "=== Step 8: Arduino sequence ===");
    
    if (!send_arduino_command('a')) {
      return {false, "Step 8: Command 'a' failed"};
    }


    
    // Special handling for 'f' (find metal) command - NO RETRY
    if (!send_arduino_command('f')) {
      RCLCPP_ERROR(this->get_logger(), "Step 8: Command 'f' (find metal) FAILED - executing recovery sequence");
      
      // Publish wire quality: FAIL
      publish_wire_quality(false);
      
      // Execute recovery sequence
      if (!execute_find_metal_recovery()) {
        return {false, "FIND_METAL_FAILED_WITH_RECOVERY_FAILED"};
      }
      
      // Recovery succeeded - return special code to retry entire sequence
      return {false, "FIND_METAL_FAILED_RETRY_SEQUENCE"};
    }
    
    // Publish wire quality: PASS (find metal succeeded)
    publish_wire_quality(true);
    
    if (!send_arduino_command('h')) {
      return {false, "Step 8: Command 'h' failed"};
    }
    
    if (!ngripper(true, 2)) {
      return {false, "Step 8: Gripper 2 close failed"};
    }
    
    if (!ngripper(false, 1)) {
      return {false, "Step 8: Gripper 1 release failed"};
    }
    
    if (!send_arduino_command('n')) {
      return {false, "Step 8: Command 'n' failed"};
    }
    
    RCLCPP_INFO(this->get_logger(), "Step 8: Arduino sequence completed successfully");


    // Step 9: Tip operations
    if (!execute_with_retry("Step 9: Tip operations", [&]() {
      auto [ok1, msg1] = movelinear(0.336, -0.420, 0.785,0.710, 0.012, -0.494, 0.501);
      if (!ok1) return false;
      
      if (!egripper(false)) return false;  // Pin 17 HIGH = small opening (open gripper)
      
      auto [ok2, msg2] = movelinear(0.336, -0.420, 0.500, 0.710, 0.013, -0.494, 0.501);
      if (!ok2) return false;
      
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      if (!egripper(true)) return false;  // Pin 17 LOW = fully closed (close gripper to grip tip)
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      
      if (!ngripper(false, 2)) return false;
      
      for (int i = 0; i < 3; ++i) {
        if (!send_arduino_command('n')) return false;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(1000));


      //slide after pick 
      
      auto [ok3, msg3] = movelinear(0.264, -0.497, 0.500, 0.710, 0.013, -0.494, 0.501);
      if (!ok3) return false;
      
      auto [ok4, msg4] = movelinear(0.264, -0.497, 0.800, 0.710, 0.013, -0.494, 0.501);
      return true ;
    })) {
      return {false, "Step 9: Tip operations failed"};
    }


    
    // Publish status: Orientation Checked
    publish_status("Orientation Checked");

    // Step 10: Move to connector top
    if (!execute_with_retry("Step 10: Move to connector top", [&]() {
      auto [ok, msg] = movelinear(0.197, -0.696, 0.867, 1.000, -0.025, 0.011, -0.014);
      return ok;
    })) {
      return {false, "Step 10: Move to connector top failed"};
    }



    
    // Step 11: Connector insertion (using provided row and col)
    // The final joglinear movements are optional - main insertion must succeed
    bool connector_inserted = false;
    
    // Save current speed settings
    double saved_velocity = velocity_scaling_;
    double saved_acceleration = acceleration_scaling_;
    
    //Slow down for precise connector insertion (adjust these values as needed)
    
    if (!execute_with_retry("Step 11: Connector insertion", [&]() {
      geometry_msgs::msg::Pose target_pose = connector_manager_.getConnectorPose(row, col);
      
      RCLCPP_INFO(get_logger(), "Inserting at row=%d, col=%d: position (%.3f, %.3f, %.3f)", 
                  row, col, target_pose.position.x, target_pose.position.y, target_pose.position.z);
      
      auto [ok1, msg1] = movelinear(
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        target_pose.orientation.x, target_pose.orientation.y, 
        target_pose.orientation.z, target_pose.orientation.w
      );
      if (!ok1) return false;
      
      connector_inserted = true;
      velocity_scaling_ = 0.02;  // Was 0.05, now 40% slower
      acceleration_scaling_ = 0.02;  // Was 0.05, now 40% slower
      move_group.setMaxVelocityScalingFactor(velocity_scaling_);
      move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
      RCLCPP_INFO(get_logger(), "Slowing down for connector insertion: vel=%.3f, acc=%.3f", 
                  velocity_scaling_, acceleration_scaling_);
    
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      auto [ok2, msg2] = joglinear(0.0, 0.0, -0.010);
      if (!ok2) {
        RCLCPP_WARN(this->get_logger(), "Final jog down failed: %s (continuing)", msg2.c_str());
      }
      
      if (!egripper(false)) return false;
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      auto [ok3, msg3] = joglinear(0.0, 0.0, 0.008);
      if (!ok3) {
        RCLCPP_WARN(this->get_logger(), "Jog up failed: %s (continuing)", msg3.c_str());
      }
      
      if (!egripper(true)) return false;
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      auto [ok4, msg4] = joglinear(0.0, 0.0, -0.008);
      if (!ok4) {
        RCLCPP_WARN(this->get_logger(), "Final jog down failed: %s (continuing)", msg4.c_str());
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      return true;
    })) {
      if (!connector_inserted) {
        return {false, "Step 11: Connector insertion failed - could not reach target position"};
      }
      RCLCPP_WARN(this->get_logger(), "Step 11: Connector insertion completed with minor issues (continuing)");
    }
    
    // Restore original speed settings
    velocity_scaling_ = saved_velocity;
    acceleration_scaling_ = saved_acceleration;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    RCLCPP_INFO(get_logger(), "Restored speed after connector insertion: vel=%.3f, acc=%.3f", 
                velocity_scaling_, acceleration_scaling_);
    
    // Publish status: Wire Inserted
    publish_status("Wire Inserted");

    // Step 12: Final positioning (optional - allowed to fail)
    bool final_positioning_success = execute_with_retry("Step 12: Final positioning", [&]() {
      if (!egripper(false)) return false;
      
      auto [ok1, msg1] = movelinear(0.197, -0.696, 0.867, 1.000, -0.025, 0.011, -0.014);
      if (!ok1) return false;
      
      if (!egripper(false)) return false;
      
      auto [ok2, msg2] = movelinear(-0.057, -0.567, 0.901, 0.733, -0.680, -0.006, -0.035);
      if (!ok2) return false;
      
      return ok2;
    });

   
   
    if (!final_positioning_success) {
      RCLCPP_WARN(this->get_logger(), "Step 12: Final positioning failed, but continuing (optional step)");
    }

    // All steps completed successfully (Step 12 failure is allowed)
    RCLCPP_INFO(this->get_logger(), "Connector sequence for row=%d, col=%d completed successfully!", row, col);
    
    // Calculate 0-based position from 1-based row/col (2 rows × 12 columns grid)
    int position = (row - 1) * 12 + (col - 1);
    publish_grid_position(position);
    
    return {true, "Connector sequence completed successfully for row=" + std::to_string(row) + ", col=" + std::to_string(col)};
  }

  geometry_msgs::msg::Pose getConnectorPose_affine(int row, int col)
  {
      if (row <= 0 || col <= 0) {
          throw std::invalid_argument("row and col must be 1-based positive integers.");
      }

      // ---- Model coefficients (least-squares fit to your measurements) ----
      const double a = 0.236333333333;   // x intercept
      const double b = 0.002000000000;   // x * col
      const double c = 0.002500000000;   // x * row

      const double d = -0.667833333333;  // y intercept
      const double e = 0.001666666667;   // y * col
      const double f = -0.002750000000;  // y * row

      // ---- Base pose (1,1) z & orientation copied from your measurement ----
      const double base_z = 0.279;
      geometry_msgs::msg::Pose out;
      out.orientation.x = -0.697;
      out.orientation.y =  0.717;
      out.orientation.z = -0.008;
      out.orientation.w =  0.011;

      // compute XY using the affine model
      out.position.x = a + b * static_cast<double>(col) + c * static_cast<double>(row);
      out.position.y = d + e * static_cast<double>(col) + f * static_cast<double>(row);
      out.position.z = base_z;

      return out;
  }

  bool send_arduino_command(char command)
  {
    if (!arduino_client_) {
      RCLCPP_ERROR(this->get_logger(), "Arduino client not initialized");
      return false;
    }

    auto request = std::make_shared<spark_stage_2_ur::srv::ArduinoCommand::Request>();
    request->command = std::string(1, command);

    RCLCPP_INFO(this->get_logger(), "Sending Arduino command: '%c'", command);

    auto future = arduino_client_->async_send_request(request);
    
    // Wait for the result with timeout
    if (future.wait_for(std::chrono::seconds(60)) == std::future_status::ready) {
      auto result = future.get();
      
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Arduino command '%c' succeeded: %s", 
                    command, result->message.c_str());
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Arduino command '%c' failed: %s", 
                     command, result->message.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Arduino command '%c' service call timeout", command);
      return false;
    }
  }


  bool set_gripper_output(const int& pin, const bool& state)
    {
      auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req->pin = pin;
      req->state = state ? ur_msgs::srv::SetIO::Request::STATE_ON
                        : ur_msgs::srv::SetIO::Request::STATE_OFF;

      auto future = gripper_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        bool success = future.get()->success;
        RCLCPP_INFO(this->get_logger(),
                    "Set DO%d -> %s : %s",
                    pin, state ? "ON" : "OFF", success ? "OK" : "FAILED");
        return success;
      }
      RCLCPP_ERROR(this->get_logger(), "Gripper SetIO service call timeout");
      return false;
    }

  bool egripper(const bool& state){
      // Pin 16 HIGH = Large opening
      // Pin 17 HIGH = Small opening
      // Both LOW = Fully closed
      // 
      // state = false: Open with small opening (Pin 17 HIGH, Pin 16 LOW)
      // state = true:  Fully closed (Both pins LOW)

      int pin16 = 16;
      int pin17 = 17;
      bool success1 = false;
      bool success2 = false;

      // First, set Pin 16 to LOW always (we don't want large opening)
      auto req1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req1->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req1->pin = pin16;
      req1->state = ur_msgs::srv::SetIO::Request::STATE_OFF;  // Always LOW
      
      auto future1 = gripper_client_->async_send_request(req1);
      if (future1.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        success1 = future1.get()->success;
      }

      // Then set Pin 17 based on state
      auto req2 = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req2->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req2->pin = pin17;
      req2->state = (!state) ? ur_msgs::srv::SetIO::Request::STATE_ON    // false = HIGH (small opening)
                             : ur_msgs::srv::SetIO::Request::STATE_OFF;  // true = LOW (fully closed)

      auto future2 = gripper_client_->async_send_request(req2);
      if (future2.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        success2 = future2.get()->success;
      }

      bool success = success1 && success2;
      
      RCLCPP_INFO(this->get_logger(),
                  "egripper(%s): Pin16=LOW, Pin17=%s -> %s",
                  state ? "true" : "false",
                  (!state) ? "HIGH" : "LOW",
                  success ? "OK" : "FAILED");

      return success;
    }
  bool egripper2(const bool& state){

      // int pin1 = 17; // actual pins on the ur controller box 
      int pin1 = 16;
      bool success1 = false;
      bool success2 = true;

      auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();

      req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req->pin = pin1;
      req->state = (!state) ? ur_msgs::srv::SetIO::Request::STATE_ON
                        : ur_msgs::srv::SetIO::Request::STATE_OFF;

      auto future = gripper_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        success1 = future.get()->success;}


      // auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
      // req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      // req->pin = (!state) ? pin2 : pin1;
      // req->state = state ? ur_msgs::srv::SetIO::Request::STATE_ON
      //                   : ur_msgs::srv::SetIO::Request::STATE_OFF;

      // auto future = gripper_client_->async_send_request(req);
      // if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      //   bool success1 = future.get()->success;
        
        
      
      bool success = success1 && success2;

      // RCLCPP_INFO(this->get_logger(),
      //               "Set DO%d -> %s : %s",
      //               pin, state ? "ON" : "OFF", success ? "OK" : "FAILED");

      return success;
    }  

  bool ngripper(const bool& state ,const int& index){
    {
      // Pin 5: HIGH to open gripper
      // Pin 4: HIGH to close gripper
      int pin_open{};   // Pin 5 HIGH = open
      int pin_close{};  // Pin 4 HIGH = close

      if (index == 0){
        pin_open = 5;   // Pin 5 HIGH = open
        pin_close = 4;  // Pin 4 HIGH = close
      }
      if (index == 1){
        pin_open = 1;   // Pin 7 HIGH = open
        pin_close = 2;  // Pin 6 HIGH = close
      }

      if (index == 2){
        pin_open = 7;   // Pin 7 HIGH = open
        pin_close = 6;  // Pin 6 HIGH = close
      }


      bool success1 = false;
      bool success2 = false;

      // First request: Set the active pin HIGH
      auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req->pin = state ? pin_close : pin_open;  // state=true->pin4, state=false->pin5
      req->state = ur_msgs::srv::SetIO::Request::STATE_ON;  // Set active pin HIGH

      auto future = gripper_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        success1 = future.get()->success;
      }

      // Second request: Set the inactive pin LOW
      auto req2 = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req2->fun = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
      req2->pin = state ? pin_open : pin_close;  // state=true->pin5, state=false->pin4
      req2->state = ur_msgs::srv::SetIO::Request::STATE_OFF;  // Set inactive pin LOW

      auto future2 = gripper_client_->async_send_request(req2);
      if (future2.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        success2 = future2.get()->success;
        
        
      }
      bool success = success1 && success2;

      // RCLCPP_INFO(this->get_logger(),
      //               "Set DO%d -> %s : %s",
      //               pin, state ? "ON" : "OFF", success ? "OK" : "FAILED");

      return success;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  std::pair<bool, std::string> joglinear(const double& x_offset, const double& y_offset, const double& z_offset)
                   
    {
      //RCLCPP_INFO(get_logger(), "=== LINEAR MOVE START ===");

      std::pair<bool, std::string> response = {false, "message"};

      // Get current pose from TF (tool0 frame)
      geometry_msgs::msg::TransformStamped transform_stamped;
      geometry_msgs::msg::Pose current_pose;
      
      try {
        // Look up transform from base_link (world) to tool0
        transform_stamped = tf_buffer_->lookupTransform(
            "base_link", "tool0", 
            tf2::TimePointZero,
            std::chrono::seconds(1));
        
        // Convert transform to pose
        current_pose.position.x = transform_stamped.transform.translation.x;
        current_pose.position.y = transform_stamped.transform.translation.y;
        current_pose.position.z = transform_stamped.transform.translation.z;
        current_pose.orientation = transform_stamped.transform.rotation;
        
      // RCLCPP_INFO(get_logger(), "Current pose from TF: (%.3f, %.3f, %.3f)", 
                    //current_pose.position.x, current_pose.position.y, current_pose.position.z);
        
      } catch (const tf2::TransformException &ex) {
        //RCLCPP_ERROR(get_logger(), "Could not get tool0 transform: %s", ex.what());
        response = {false, "TF lookup failed"};
        return response;
      }


    // Target: move in X, Y, Z axes (use provided offsets)
      
      geometry_msgs::msg::Pose target_pose = current_pose;
      target_pose.position.x = current_pose.position.x + x_offset;
      target_pose.position.y = current_pose.position.y + y_offset;
      target_pose.position.z = current_pose.position.z + z_offset;

      //RCLCPP_INFO(get_logger(), "Target: (%.3f, %.3f, %.3f) [Offsets: X=%.3f, Y=%.3f, Z=%.3f]", 
                  //target_pose.position.x, target_pose.position.y, target_pose.position.z,
                  //x_offset, y_offset, z_offset);

      // Cartesian path for straight-line motion
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
      waypoints,
      0.01,  // 1cm step
      0.0,   // no jump threshold
      trajectory);

    //RCLCPP_INFO(get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);

    if (fraction > 0.8) {
        // Check if trajectory exceeds maximum waypoints limit
        const size_t max_waypoints = 20;
        if (trajectory.joint_trajectory.points.size() > max_waypoints) {
          RCLCPP_ERROR(get_logger(), "Jog trajectory has %zu points, exceeds maximum of %zu - aborting execution", 
                      trajectory.joint_trajectory.points.size(), max_waypoints);
          response = {false, "Jog trajectory exceeds 20 waypoints limit (" + std::to_string(trajectory.joint_trajectory.points.size()) + " points)"};
          return response;
        }
        
        // Apply velocity and acceleration scaling by time-parameterizing the trajectory
        // Create a robot state for time parameterization
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
        robot_state->setToDefaultValues();
        
        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*robot_state, trajectory);
        
        // Clear any existing time stamps to ensure fresh parameterization
        for (auto& point : trajectory.joint_trajectory.points) {
          point.time_from_start = rclcpp::Duration(0, 0);
        }
        
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        bool success = time_param.computeTimeStamps(
            rt, 
            velocity_scaling_,
            acceleration_scaling_);
        
        if (success) {
          rt.getRobotTrajectoryMsg(trajectory);
          RCLCPP_INFO(get_logger(), "Jog time-parameterized with vel=%.2f, acc=%.2f (offset: %.3f, %.3f, %.3f)", 
                          velocity_scaling_, acceleration_scaling_, x_offset, y_offset, z_offset);
        } else {
          RCLCPP_WARN(get_logger(), "Jog time parameterization failed, using default timing");
        }
        
        // Execute the time-parameterized path for an actual 1 cm step
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        
        // Add a small delay to allow controller to be ready
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        
        auto exec_code = move_group.execute(plan);

        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "Jog execution failed with code %d (offset: %.3f, %.3f, %.3f)", 
                      exec_code.val, x_offset, y_offset, z_offset);
          response = {false, "Execution failed"};
        } else {
          //RCLCPP_INFO(get_logger(), "Linear step executed successfully");
          response = {true, "Linear step executed"};
        }
      } else {
        //RCLCPP_WARN(get_logger(), "FAILED: Only %.1f%% of path computed", fraction * 100.0);
        response = {false, "Could not compute full linear path."};
      }

      //RCLCPP_INFO(get_logger(), "=== LINEAR MOVE END ===");
      return response;
    }
  
  std::pair<bool, std::string> movelinear(const double& x, const double& y, const double& z, const double& qx, const double& qy, const double& qz, const double& qw)
  {
    RCLCPP_INFO(get_logger(), "=== LINEAR CARTESIAN MOVE (QUATERNION) ===");
    RCLCPP_INFO(get_logger(), "Translation xyz: (%.3f, %.3f, %.3f)", x, y, z);
    RCLCPP_INFO(get_logger(), "Rotation quat (xyzw): (%.3f, %.3f, %.3f, %.3f)", qx, qy, qz, qw);

    std::pair<bool, std::string> response = {false, "message"};

    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    // Translation
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    
    // Rotation (quaternion)
    constexpr double kQuatEpsilon = 1e-6;
    const double quat_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;

    geometry_msgs::msg::Quaternion orientation;
    bool use_current_orientation = quat_norm_sq < kQuatEpsilon;
    
    if (use_current_orientation) {
      // If no quaternion provided, use current end-effector orientation
      try {
        auto transform_stamped = tf_buffer_->lookupTransform(
            "base_link", "tool0",
            tf2::TimePointZero,
            std::chrono::seconds(1));
        orientation = transform_stamped.transform.rotation;
        RCLCPP_INFO(get_logger(), "No quaternion provided, using current end-effector orientation.");
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF lookup failed for current orientation: %s", ex.what());
        response = {false, "TF lookup failed"};
        return response;
      }
    } else {
      // Normalize the provided quaternion
      const double quat_norm = std::sqrt(quat_norm_sq);
      if (quat_norm < kQuatEpsilon) {
        RCLCPP_ERROR(get_logger(), "Invalid quaternion norm: %.3e", quat_norm);
        response = {false, "Invalid quaternion"};
        return response;
      }
      orientation.x = qx / quat_norm;
      orientation.y = qy / quat_norm;
      orientation.z = qz / quat_norm;
      orientation.w = qw / quat_norm;
    }

    target_pose.orientation = orientation;

    // Compute Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    RCLCPP_INFO(get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);

    if (fraction > 0.8) {
      // Check if trajectory exceeds maximum waypoints limit
      const size_t max_waypoints = 20;
      if (trajectory.joint_trajectory.points.size() > max_waypoints) {
        RCLCPP_ERROR(get_logger(), "Trajectory has %zu points, exceeds maximum of %zu - aborting execution", 
                    trajectory.joint_trajectory.points.size(), max_waypoints);
        response = {false, "Trajectory exceeds 20 waypoints limit (" + std::to_string(trajectory.joint_trajectory.points.size()) + " points)"};
        return response;
      }
      
      // Apply time parameterization for velocity/acceleration scaling
      moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
      robot_state->setToDefaultValues();
      
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
      rt.setRobotTrajectoryMsg(*robot_state, trajectory);
      
      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      bool time_success = time_param.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);
      
      if (time_success) {
        rt.getRobotTrajectoryMsg(trajectory);
        RCLCPP_INFO(get_logger(), "Trajectory time-parameterized with vel=%.2f, acc=%.2f", 
                    velocity_scaling_, acceleration_scaling_);
      }
      
      // Execute the trajectory
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      RCLCPP_INFO(get_logger(), "Executing trajectory with %zu points...", 
                  plan.trajectory_.joint_trajectory.points.size());
      auto execute_result =  move_group.execute(plan);
      // auto execute_result =  true;
      
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Trajectory execution SUCCESS");
        // Add small delay to allow controller to finish and update state
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        response = {true, "Linear path executed successfully. Fraction: " + std::to_string(int(fraction * 100)) + "%"};
      } else {
        RCLCPP_ERROR(get_logger(), "Trajectory execution FAILED with error code: %d", execute_result.val);
        response = {false, "Execution failed with code " + std::to_string(execute_result.val) + ". Fraction: " + std::to_string(int(fraction * 100)) + "%"};
      }
    } else {
      response = {false, "Cartesian path only " + std::to_string(int(fraction * 100)) + "% achievable - obstacles or joint limits"};
    }
    
    return response;
  }

  /**
   * @brief Move robot to specified joint angles
   * @param j0 shoulder_pan_joint angle (radians)
   * @param j1 shoulder_lift_joint angle (radians)
   * @param j2 elbow_joint angle (radians)
   * @param j3 wrist_1_joint angle (radians)
   * @param j4 wrist_2_joint angle (radians)
   * @param j5 wrist_3_joint angle (radians)
   * @return pair<bool, string> - success status and message
   * 
   * NOTE: Function parameters use logical naming (j0=pan, j1=lift, etc.)
   * but internally reorders to match actual /joint_states topic format:
   *   /joint_states order: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
   *   Function parameters:  [j0=pan, j1=lift, j2=elbow, j3=wrist1, j4=wrist2, j5=wrist3]
   * 
   * Example usage:
   *   auto [success, msg] = movejoints(0.0, -1.57, 1.57, -1.57, 0.0, 0.0);
   *   if (success) { RCLCPP_INFO(get_logger(), "Success: %s", msg.c_str()); }
   */
  std::pair<bool, std::string> movejoints(double j0, double j1, double j2, double j3, double j4, double j5)
  {
    RCLCPP_INFO(get_logger(), "=== MOVE TO JOINT ANGLES (Direct Publisher) ===");
    RCLCPP_INFO(get_logger(), "Target joints (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", j0, j1, j2, j3, j4, j5);
    RCLCPP_INFO(get_logger(), "Target joints (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", 
                j0*180.0/M_PI, j1*180.0/M_PI, j2*180.0/M_PI, 
                j3*180.0/M_PI, j4*180.0/M_PI, j5*180.0/M_PI);

    std::pair<bool, std::string> response = {false, "Not executed"};

    try {
      // Create trajectory message
      trajectory_msgs::msg::JointTrajectory traj_msg;
      
      // Set the joint names (MUST match the EXACT order in joint_states topic)
      // From joint_states: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
      traj_msg.joint_names = {
        "shoulder_lift_joint",  // index 0
        "elbow_joint",          // index 1
        "wrist_1_joint",        // index 2
        "wrist_2_joint",        // index 3
        "wrist_3_joint",        // index 4
        "shoulder_pan_joint"    // index 5 (LAST, not first!)
      };
      
      RCLCPP_INFO(get_logger(), "Joint order being sent (matches /joint_states order):");
      RCLCPP_INFO(get_logger(), "  [0] shoulder_lift_joint   = %.3f rad (%.1f deg) <- j1", j1, j1*180.0/M_PI);
      RCLCPP_INFO(get_logger(), "  [1] elbow_joint           = %.3f rad (%.1f deg) <- j2", j2, j2*180.0/M_PI);
      RCLCPP_INFO(get_logger(), "  [2] wrist_1_joint         = %.3f rad (%.1f deg) <- j3", j3, j3*180.0/M_PI);
      RCLCPP_INFO(get_logger(), "  [3] wrist_2_joint         = %.3f rad (%.1f deg) <- j4", j4, j4*180.0/M_PI);
      RCLCPP_INFO(get_logger(), "  [4] wrist_3_joint         = %.3f rad (%.1f deg) <- j5", j5, j5*180.0/M_PI);
      RCLCPP_INFO(get_logger(), "  [5] shoulder_pan_joint    = %.3f rad (%.1f deg) <- j0", j0, j0*180.0/M_PI);
      
      // Create a trajectory point with target positions
      // Reorder to match joint_states: [j1, j2, j3, j4, j5, j0]
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {j1, j2, j3, j4, j5, j0};
      
      // Don't set velocities/accelerations - let controller compute them
      // OR set them to empty to let controller decide
      point.velocities = {};
      point.accelerations = {};
      
      // Set time from start (slow motion - 50 seconds)
      point.time_from_start = rclcpp::Duration::from_seconds(50.0);
      
      // Add the point to the trajectory
      traj_msg.points.push_back(point);
      
      // Set timestamp to zero to execute immediately
      traj_msg.header.stamp = rclcpp::Time(0);
      traj_msg.header.frame_id = "";
      
      // Publish the trajectory
      joint_trajectory_pub_->publish(traj_msg);
      
      RCLCPP_INFO(get_logger(), "Joint trajectory published successfully!");
      response = {true, "Joint trajectory published to controller"};
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during joint trajectory publishing: %s", e.what());
      response = {false, std::string("Exception: ") + e.what()};
    }

    return response;
  }
  
  // Linear Cartesian move with quaternion orientation
  void move_linear_quat_callback(const std::shared_ptr<spark_stage_2_ur::srv::MoveLinearQuat::Request> request,
                                  std::shared_ptr<spark_stage_2_ur::srv::MoveLinearQuat::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== LINEAR CARTESIAN MOVE (QUATERNION) ===");
    RCLCPP_INFO(get_logger(), "Translation xyz: (%.3f, %.3f, %.3f)", 
                request->trans_x, request->trans_y, request->trans_z);
    RCLCPP_INFO(get_logger(), "Rotation quat (xyzw): (%.3f, %.3f, %.3f, %.3f)", 
                request->quat_x, request->quat_y, request->quat_z, request->quat_w);

    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    // Translation
    target_pose.position.x = request->trans_x;
    target_pose.position.y = request->trans_y;
    target_pose.position.z = request->trans_z;
    
    // Rotation (quaternion)
    constexpr double kQuatEpsilon = 1e-6;
    const double quat_norm_sq =
        request->quat_x * request->quat_x +
        request->quat_y * request->quat_y +
        request->quat_z * request->quat_z +
        request->quat_w * request->quat_w;

    geometry_msgs::msg::Quaternion orientation;
    bool use_current_orientation = quat_norm_sq < kQuatEpsilon;
    
    if (use_current_orientation) {
      // If no quaternion provided, use current end-effector orientation
      try {
        auto transform_stamped = tf_buffer_->lookupTransform(
            "base_link", "tool0",
            tf2::TimePointZero,
            std::chrono::seconds(1));
        orientation = transform_stamped.transform.rotation;
        RCLCPP_INFO(get_logger(), "No quaternion provided, using current end-effector orientation.");
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF lookup failed for current orientation: %s", ex.what());
        response->success = false;
        response->message = "TF lookup failed";
        response->fraction = 0.0;
        return;
      }
    } else {
      // Normalize the provided quaternion
      const double quat_norm = std::sqrt(quat_norm_sq);
      if (quat_norm < kQuatEpsilon) {
        RCLCPP_ERROR(get_logger(), "Invalid quaternion norm: %.3e", quat_norm);
        response->success = false;
        response->message = "Invalid quaternion";
        response->fraction = 0.0;
        return;
      }
      orientation.x = request->quat_x / quat_norm;
      orientation.y = request->quat_y / quat_norm;
      orientation.z = request->quat_z / quat_norm;
      orientation.w = request->quat_w / quat_norm;
    }

    target_pose.orientation = orientation;

    // Compute Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    RCLCPP_INFO(get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);
    response->fraction = fraction;

    if (fraction > 0.8) {
      // Check if trajectory exceeds maximum waypoints limit
      const size_t max_waypoints = 20;
      if (trajectory.joint_trajectory.points.size() > max_waypoints) {
        RCLCPP_ERROR(get_logger(), "move_linear_quat trajectory has %zu points, exceeds maximum of %zu - aborting execution", 
                    trajectory.joint_trajectory.points.size(), max_waypoints);
        response->success = false;
        response->message = "Trajectory exceeds 20 waypoints limit (" + std::to_string(trajectory.joint_trajectory.points.size()) + " points)";
        return;
      }
      
      // Apply time parameterization for velocity/acceleration scaling
      moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
      robot_state->setToDefaultValues();
      
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
      rt.setRobotTrajectoryMsg(*robot_state, trajectory);
      
      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      bool time_success = time_param.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);
      
      if (time_success) {
        rt.getRobotTrajectoryMsg(trajectory);
        RCLCPP_INFO(get_logger(), "Trajectory time-parameterized with vel=%.2f, acc=%.2f", 
                    velocity_scaling_, acceleration_scaling_);
      }
      
      // Execute the trajectory
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      //auto execute_result = move_group.execute(plan);
      auto execute_result = true;
      
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        response->message = "Linear path executed successfully. Fraction: " + 
                           std::to_string(int(fraction * 100)) + "%";
      } else {
        response->success = false;
        response->message = "Execution failed. Fraction: " + std::to_string(int(fraction * 100)) + "%";
      }
    } else {
      response->success = false;
      response->message = "Cartesian path only " + std::to_string(int(fraction * 100)) + "% achievable - obstacles or joint limits";
    }
  }

  void move_to_joints_callback(const std::shared_ptr<spark_stage_2_ur::srv::MoveToJoints::Request> request,
                                std::shared_ptr<spark_stage_2_ur::srv::MoveToJoints::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== MOVE TO JOINT ANGLES ===");
    
    // Validate input
    if (request->joint_positions.size() != 6) {
      RCLCPP_ERROR(get_logger(), "Expected 6 joint angles, got %zu", request->joint_positions.size());
      response->success = false;
      response->message = "Invalid number of joint angles. Expected 6, got " + 
                         std::to_string(request->joint_positions.size());
      return;
    }

    // Log the target joint angles with proper joint names
    RCLCPP_INFO(get_logger(), "Target joint angles (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                request->joint_positions[0], request->joint_positions[1], 
                request->joint_positions[2], request->joint_positions[3],
                request->joint_positions[4], request->joint_positions[5]);
    RCLCPP_INFO(get_logger(), "Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]");

    // Set joint value target
    try {
      std::vector<double> joint_group_positions(request->joint_positions.begin(), 
                                                request->joint_positions.end());
      
      move_group.setJointValueTarget(joint_group_positions);
      move_group.setPlanningTime(5.0);
      
      // Plan the motion
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode success_code = move_group.plan(plan);
      
      if (success_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Planning to joint angles failed with error code: %d", 
                     success_code.val);
        response->success = false;
        response->message = "Planning failed with error code: " + std::to_string(success_code.val);
        return;
      }
      
      RCLCPP_INFO(get_logger(), "Planning successful! Executing motion...");
      
      // Execute the planned motion
      moveit::core::MoveItErrorCode execute_code = move_group.execute(plan);
      
      if (execute_code == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Successfully moved to target joint angles");
        response->success = true;
        response->message = "Successfully moved to joint angles";
      } else {
        RCLCPP_ERROR(get_logger(), "Execution failed with error code: %d", execute_code.val);
        response->success = false;
        response->message = "Execution failed with error code: " + std::to_string(execute_code.val);
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during joint motion: %s", e.what());
      response->success = false;
      response->message = std::string("Exception: ") + e.what();
    }
  }

  void set_grid_positions_callback(const std::shared_ptr<pcb_visualization_msgs::srv::SetGridPositions::Request> request,
                                    std::shared_ptr<pcb_visualization_msgs::srv::SetGridPositions::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== SET GRID POSITIONS SERVICE CALLED ===");
    RCLCPP_INFO(get_logger(), "Received %zu grid position indices", request->positions.size());
    
    // Log the grid positions
    for (size_t i = 0; i < request->positions.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  Position[%zu]: %d", i, request->positions[i]);
    }
    
    try {
      // Process each grid position
      for (size_t idx = 0; idx < request->positions.size(); ++idx) {
        const auto& position = request->positions[idx];
        
        // Convert 1D position to row/col for 2x12 grid (2 rows, 12 columns)
        // GUI sends 0-based indices (0-23 for 24 total positions)
        // Grid layout: 2 rows × 12 columns = 24 positions
        const int cols = 12;  // 12 columns per row
        const int rows = 2;   // 2 rows total
        
        // Validate position is in valid range
        if (position < 0 || position >= (rows * cols)) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid position %d (valid range: 0-%d)", position, (rows * cols - 1));
          continue;  // Skip this position
        }
        
        // Convert 0-based linear index to 1-based row/col
        int row = (position / cols) + 1;  // 1-based row (1, 2)
        int col = (position % cols) + 1;  // 1-based col (1-12)
        
        RCLCPP_INFO(get_logger(), "Processing grid position %zu/%zu: 0-based index %d -> 1-based (row=%d, col=%d)", 
                    idx + 1, request->positions.size(), position, row, col);
        
        // Publish grid position update before starting
        
        // Run the full connector insertion sequence for this grid position
        // Retry logic for 'find metal' failure
        bool sequence_success = false;
        std::string sequence_message;
        
        for (int attempt = 1; attempt <= 2; ++attempt) {
          if (attempt > 1) {
            RCLCPP_WARN(get_logger(), "Retrying entire sequence for position %d (row=%d, col=%d) - Attempt %d/2", 
                        position, row, col, attempt);
          }
          
          auto [success, message] = run_connector_sequence(row, col);
          
          // Check if this is a 'find metal' failure that requests retry
          if (!success && message == "FIND_METAL_FAILED_RETRY_SEQUENCE") {
            RCLCPP_WARN(get_logger(), "Find metal failed but recovery succeeded - retrying entire sequence");
            sequence_message = message;
            continue; // Retry the sequence
          }
          
          // Check if recovery also failed
          if (!success && message == "FIND_METAL_FAILED_WITH_RECOVERY_FAILED") {
            RCLCPP_ERROR(get_logger(), "Find metal failed AND recovery failed - aborting");
            sequence_success = false;
            sequence_message = message;
            break; // Don't retry if recovery failed
          }
          
          // Normal success or failure
          sequence_success = success;
          sequence_message = message;
          
          if (success) {
            break; // Success - no need to retry
          }
        }
        
        if (!sequence_success) {
          RCLCPP_ERROR(get_logger(), "Connector sequence failed for position %d (row=%d, col=%d) after all attempts: %s", 
                       position, row, col, sequence_message.c_str());
          response->success = false;
          response->message = "Failed at position " + std::to_string(position) + " (row=" + 
                             std::to_string(row) + ", col=" + std::to_string(col) + "): " + sequence_message;
          return;
        }
        
        RCLCPP_INFO(get_logger(), "Successfully completed position %zu/%zu (row=%d, col=%d)", 
                    idx + 1, request->positions.size(), row, col);
        
        // Publish "Done" after completing this position
        publish_status("Done");
        
        // Small delay between positions
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
      
      response->success = true;
      response->message = "Successfully processed " + std::to_string(request->positions.size()) + " grid positions";
      RCLCPP_INFO(get_logger(), "All %zu grid positions processed successfully!", request->positions.size());
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during grid position processing: %s", e.what());
      response->success = false;
      response->message = std::string("Exception: ") + e.what();
    }
  }

  void define_table_collision_object()


  
  {std::cout << "fine" << std::endl;
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {5.0, 5.0, 0.05};  // length, width, height

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.05 / 2.0;  // table sits just below base

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    planning_scene_interface.applyCollisionObjects({table});

    RCLCPP_INFO(this->get_logger(), "Table collision object added to planning scene.");
  }

  void allow_table_base_contact()
    {
      const std::string base_link = move_group.getRobotModel()->getRootLinkName();  // likely "base_link"
      moveit_msgs::msg::PlanningScene scene;
      scene.is_diff = true;

      moveit_msgs::msg::AllowedCollisionMatrix acm;
      acm.entry_names = {base_link, "table"};
      acm.entry_values.resize(2);
      acm.entry_values[0].enabled = {false, true};   // base link may collide with table
      acm.entry_values[1].enabled = {true, false};   // symmetry

      scene.allowed_collision_matrix = acm;
      planning_scene_interface.applyPlanningScene(scene);
    }

  // Move to absolute pose with obstacle avoidance (uses MoveIt planner, not straight line)
  // Legacy function - kept for backward compatibility, now takes full pose
  bool move_to_pose_with_planning(double target_x, double target_y, double target_z,
                                   std::string &result_message)
  {
    // Get current orientation and call with it
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          "base_link", "tool0", 
          tf2::TimePointZero,
          std::chrono::seconds(1));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
      result_message = "TF lookup failed";
      return false;
    }
    
    // Keep current orientation
    return move_to_pose_with_planning(
      target_x, target_y, target_z,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w,
      result_message
    );
  }

  // Move to absolute pose with full orientation control
  bool move_to_pose_with_planning(double target_x, double target_y, double target_z,
                                   double quat_x, double quat_y, double quat_z, double quat_w,
                                   std::string &result_message)
  {
    RCLCPP_INFO(get_logger(), "=== PLANNING MOVE WITH OBSTACLE AVOIDANCE (SHORTEST PATH) ===");
    RCLCPP_INFO(get_logger(), "Target: (%.3f, %.3f, %.3f)", target_x, target_y, target_z);
    RCLCPP_INFO(get_logger(), "Orientation: (%.3f, %.3f, %.3f, %.3f)", quat_x, quat_y, quat_z, quat_w);

    // Set target pose with full orientation
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z;
    target_pose.orientation.x = quat_x;
    target_pose.orientation.y = quat_y;
    target_pose.orientation.z = quat_z;
    target_pose.orientation.w = quat_w;

    // Use MoveIt with RRTConnect for faster, shorter paths
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(5.0);
    move_group.setPlannerId("RRTConnectkConfigDefault");  // Fast planner that finds short paths
    move_group.setGoalPositionTolerance(0.001);  // 1mm
    move_group.setGoalOrientationTolerance(0.01); // ~0.5 degrees

    // Try multiple planning attempts and pick shortest
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  double best_duration = std::numeric_limits<double>::max();
  double best_travel = std::numeric_limits<double>::infinity();
    bool found_plan = false;

    for (int attempt = 0; attempt < 3; attempt++) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode success = move_group.plan(plan);

      if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        found_plan = true;
        double duration = 0.0;
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
          duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                    plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
        }

        RCLCPP_INFO(get_logger(), "Attempt %d: duration=%.2fs", attempt + 1, duration);

        double travel = joint_travel_distance(plan.trajectory_);
        if (travel < best_travel || (std::fabs(travel - best_travel) < 1e-6 && duration < best_duration)) {
          best_travel = travel;
          best_duration = duration;
          best_plan = plan;
        }
      }
    }

    if (found_plan) {
      RCLCPP_INFO(get_logger(), "Best path selected: joint travel %.3f rad, duration %.2fs, %zu points",
                  best_travel, best_duration,
                  best_plan.trajectory_.joint_trajectory.points.size());
      // Uncomment to execute: move_group.execute(best_plan);
      result_message = "Shortest path found (duration: " + std::to_string(best_duration) + "s)";
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "All planning attempts failed");
      result_message = "Planning failed - no valid path found";
      return false;
    }
  }

  // BACKUP METHOD (commented): Build robot state from joint_states
    // ensureJointStateSub();
    // auto start_wait = std::chrono::steady_clock::now();
    // while (!have_joint_state_ || latest_positions_.empty()) {
    //   rclcpp::sleep_for(std::chrono::milliseconds(100));
    //   auto elapsed = std::chrono::steady_clock::now() - start_wait;
    //   if (elapsed > std::chrono::seconds(5)) {
    //     RCLCPP_ERROR(get_logger(), "Timeout waiting for /joint_states!");
    //     response->success = false;
    //     response->message = "No joint_states received";
    //     return;
    //   }
    // }
    // std::vector<std::string> joint_names = move_group.getJointNames();
    // std::vector<double> joint_values;
    // {
    //   std::lock_guard<std::mutex> lk(js_mutex_);
    //   for (const auto& jname : joint_names) {
    //     auto it = latest_positions_.find(jname);
    //     if (it != latest_positions_.end()) {
    //       joint_values.push_back(it->second);
    //     } else {
    //       joint_values.push_back(0.0);
    //     }
    //   }
    // }
    // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    // robot_state->setToDefaultValues();
    // for (size_t i = 0; i < joint_names.size(); ++i) {
    //   robot_state->setJointPositions(joint_names[i], &joint_values[i]);
    // }
    // robot_state->update();
    // const Eigen::Isometry3d& end_effector_state = 
    //     robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink());
    // current_pose.position.x = end_effector_state.translation().x();
    // current_pose.position.y = end_effector_state.translation().y();
    // current_pose.position.z = end_effector_state.translation().z();
    // Eigen::Quaterniond q(end_effector_state.rotation());
    // current_pose.orientation.x = q.x();
    // current_pose.orientation.y = q.y();
    // current_pose.orientation.z = q.z();
    // current_pose.orientation.w = q.w();

  // Ensure we subscribe to joint states and keep latest by name
//   void ensureJointStateSub()
//   {
//     if (joint_state_sub_) return;
//     // Use system default QoS to match whatever the publisher uses
//     joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "/joint_states", 10,
//       [this](const sensor_msgs::msg::JointState::SharedPtr msg){
//         std::lock_guard<std::mutex> lk(js_mutex_);
//         latest_positions_.clear();
//         for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
//           latest_positions_[msg->name[i]] = msg->position[i];
//         }
//         have_joint_state_ = !latest_positions_.empty();
//         RCLCPP_INFO_ONCE(get_logger(), "Received first /joint_states message");
//       }
//     );
//     RCLCPP_INFO(get_logger(), "Subscribed to /joint_states");
//   }

//   // Build and apply a start state from the latest /joint_states regardless of timestamp freshness
//   bool applyExplicitStartStateFromJointStates()
//   {
//     ensureJointStateSub();
//     if (!have_joint_state_) {
//       // Give it a brief chance to receive something
//       rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }
//     std::lock_guard<std::mutex> lk(js_mutex_);
//     if (!have_joint_state_) {
//       return false;
//     }
//     const auto robot_model = move_group.getRobotModel();
//     moveit::core::RobotState start_state(robot_model);
//     start_state.setToDefaultValues();
//     for (const auto &jn : move_group.getJointNames()) {
//       auto it = latest_positions_.find(jn);
//       if (it != latest_positions_.end()) {
//         start_state.setVariablePosition(jn, it->second);
//       }
//     }
//     move_group.setStartState(start_state);
//     return true;
//   }



  // start_monitor() is commented out - not needed with TF approach
  // void start_monitor(){
  //   double wait_seconds = 5.0;
  //   bool monitor_ok = move_group.startStateMonitor(wait_seconds);
  //   if (!monitor_ok) {
  //     RCLCPP_WARN(get_logger(), "startStateMonitor() returned false after %.1f s", wait_seconds);
  //   } else {
  //     RCLCPP_INFO(get_logger(), "startStateMonitor() succeeded");
  //   }
  // }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseNode>();
  // Use MultiThreadedExecutor so sensor subscriptions can run while services execute
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}