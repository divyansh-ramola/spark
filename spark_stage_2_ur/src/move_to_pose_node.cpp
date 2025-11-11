#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "spark_stage_2_ur/srv/move_to_pose.hpp"
#include "spark_stage_2_ur/srv/move_linear_rpy.hpp"
#include "spark_stage_2_ur/srv/move_linear_quat.hpp"
#include "spark_stage_2_ur/srv/arduino_command.hpp"
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

class MoveToPoseNode : public rclcpp::Node
{
public:
  explicit MoveToPoseNode()
  : Node("move_to_pose_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator"),
    velocity_scaling_(1.0),
    acceleration_scaling_(1.0),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    RCLCPP_INFO(this->get_logger(), "Move to Pose Node has been started.");

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

    gripper_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    while (!gripper_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /io_and_status_controller/set_io service to be available...");
    }

    arduino_client_ = create_client<spark_stage_2_ur::srv::ArduinoCommand>("arduino/send_command");
    // while (!arduino_client_->wait_for_service(std::chrono::seconds(2))) {
    //   RCLCPP_INFO(this->get_logger(), "Waiting for arduino/send_command service to be available...");
    // }

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

    
    RCLCPP_INFO(this->get_logger(), "Node ready - services available:");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose (linear Cartesian, offset-based)");
    RCLCPP_INFO(this->get_logger(), "  - /move_with_planning (obstacle avoidance, hardcoded target)");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose_full (full pose control: XYZ + quaternion XYZW)");
    RCLCPP_INFO(this->get_logger(), "  - /move_linear_rpy (linear Cartesian: XYZ + RPY angles)");
    RCLCPP_INFO(this->get_logger(), "  - /move_linear_quat (linear Cartesian: XYZ + quaternion XYZW)");
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planning_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveToPose>::SharedPtr pose_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveLinearRPY>::SharedPtr linear_rpy_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveLinearQuat>::SharedPtr linear_quat_service;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Separate callback groups so sensor callbacks can run in parallel with services
  rclcpp::CallbackGroup::SharedPtr group_sensors_;
  rclcpp::CallbackGroup::SharedPtr group_services_;

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

    // Log continuously during descent to help debug
    static int log_counter = 0;
    

    if (over_threshold && !vacuum_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "WIRE DETECTED! voltage: %.3f V, drop: %.3f V (threshold: %.3f V)", 
                  analog, voltage_drop, threshold);
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
    const bool over_threshold = ((std::abs(force.z) - wrench_initial_) > 80.0);

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
        auto [ok_offset, msg_offset] = joglinear(0.0, relative_y_move, 0.0);
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
      auto [ok_lift, msg_lift] = joglinear(0.0, 0.0, 0.020);
      if (!ok_lift) {
        RCLCPP_WARN(this->get_logger(), "Attempt %d: lift failed: %s", tries + 1, msg_lift.c_str());
      }

      if (detected) {
        // Stop outer loop on successful trigger
        joglinear(0.0, 0.0, 0.223);
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
        bool grip_ok = ngripper(false,0);
        
      }
    }  // end for loop

    return detected;
  }

  void move_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Starting descent attempts; stop when wrench or vacuum triggers, else try new Y offset");

    movelinear(0.247, -0.632, 0.569 ,-0.699, 0.715, 0.014, 0.010);
    {
      bool grip_ok = ngripper(false,0);
      if(!grip_ok){
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper DO0");
        response->success = false;
        response->message = "Failed to open gripper DO0";
        return;
      }
      
    }

    {
      bool grip_ok = ngripper(false,1);
    }
    {
      bool grip_ok = ngripper(false,2);
    }
    
    // movelinear(0.247, -0.631, 0.312, -0.699, 0.715, 0.014, 0.010);

    // // Configure attempts across Y offsets (stop only when vacuum becomes active)
    // const int no_of_tries = 7;
    // const int kMaxSteps = 200;  // safety stop per attempt
    
    // bool detected = perform_descent_with_y_scanning(no_of_tries, kMaxSteps, 0.010);


    // if(!detected){
    //   RCLCPP_ERROR(this->get_logger(), "Failed to open gripper DO0");
    //     response->success = false;
    //     response->message = "Failed to open gripper DO0";
    //     return;

    // }

    {
     // bool grip_ok = ngripper(true,0);
    };


    // movelinear(0.247, -0.632, 0.779 ,-0.699, 0.715, 0.014, 0.010); // top of the bin 
    // movelinear(0.729, -0.050, 0.706 ,0.997, 0.027, 0.016, -0.069); //above gripper1


    // movelinear(0.729, -0.050, 0.580 ,0.997, 0.027, 0.016, -0.069); //gripper1
     movelinear(0.736, -0.038, 0.706 ,0.060, 0.996, 0.068, 0.018); //atthetip
     movelinear(0.736, -0.038, 0.584 ,0.060, 0.996, 0.068, 0.018); //atthetip
    // {
    //   bool grip_ok = ngripper(true,1);
    // }

    // {
    //   bool grip_ok = ngripper(false,0);
    // }
    // movelinear(0.703, -0.012, 0.581 ,0.029, 0.998, 0.054, 0.018); //above gripper1


    // {
    //   bool grip_ok = ngripper(true,1);
    // }

    // {
    //   bool success = send_arduino_command('f');
    //   if (!success) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to send 'f' command to Arduino");
    //     response->success = false;
    //     response->message = "Failed to send 'f' command to Arduino";
    //     return;
    //   }
    // }

    // {
    //   bool grip_ok = ngripper(true,2);
    // }

    // {
    //   bool success = send_arduino_command('f');
    //   if (!success) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to send 'f' command to Arduino");
    //     response->success = false;
    //     response->message = "Failed to send 'f' command to Arduino";
    //     return;
    //   }
    // }

    // { egripper(false);
    //   joglinear(0.0, 0.0, 0.020);
    //   egripper(true);
    //   joglinear(0.0, 0.0, -0.020);
    //   egripper(false);   
    // }




   
    // if (detected) {
    //   response->success = true;
    //   response->message = "Vacuum (wire) detected during descent";
    // } else {
    //   response->success = false;
    //   response->message = "All attempts completed without vacuum trigger";
    //   joglinear(0.0, 0.0, 0.253);
    // }
    
    // return;
  }

  // Function to send command to Arduino node
  // Input: command - character command ('h', 'f', 'm', 'n')
  // Returns: true if command executed successfully, false otherwise
  bool send_arduino_command(const char& command)
  {
    if (!arduino_client_) {
      RCLCPP_ERROR(this->get_logger(), "Arduino client not initialized");
      return false;
    }

    auto request = std::make_shared<spark_stage_2_ur::srv::ArduinoCommand::Request>();
    request->command = command;

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

      int pin1 = 16; // actual pins on the ur controller box 
      //int pin2 = 17;
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
        pin_open = 6;   // Pin 7 HIGH = open
        pin_close = 7;  // Pin 6 HIGH = close
      }

      if (index == 2){
        pin_open = 1;   // Pin 7 HIGH = open
        pin_close = 2;  // Pin 6 HIGH = close
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
        // Apply velocity and acceleration scaling by time-parameterizing the trajectory
        // Create a robot state for time parameterization
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
        robot_state->setToDefaultValues();
        
        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*robot_state, trajectory);
        
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        bool success = time_param.computeTimeStamps(
            rt, 
            velocity_scaling_,
            acceleration_scaling_);
        
        if (success) {
          rt.getRobotTrajectoryMsg(trajectory);
          //RCLCPP_INFO(get_logger(), "Trajectory time-parameterized with vel=%.2f, acc=%.2f", 
          //                velocity_scaling_, acceleration_scaling_);
        } else {
          //RCLCPP_WARN(get_logger(), "Time parameterization failed, using default timing");
        }
        
        // Execute the time-parameterized path for an actual 1 cm step
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto exec_code = move_group.execute(plan);


        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
          //RCLCPP_WARN(get_logger(), "Execution failed with code %d", exec_code.val);
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
      auto execute_result = move_group.execute(plan);
      
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        response = {true, "Linear path executed successfully. Fraction: " + std::to_string(int(fraction * 100)) + "%"};
      } else {
        response = {false, "Execution failed. Fraction: " + std::to_string(int(fraction * 100)) + "%"};
      }
    } else {
      response = {false, "Cartesian path only " + std::to_string(int(fraction * 100)) + "% achievable - obstacles or joint limits"};
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