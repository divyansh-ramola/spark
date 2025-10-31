#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "spark_stage_2_ur/srv/move_to_pose.hpp"
#include "spark_stage_2_ur/srv/move_linear_rpy.hpp"
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

class MoveToPoseNode : public rclcpp::Node
{
public:
  explicit MoveToPoseNode()
  : Node("move_to_pose_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator"),
    velocity_scaling_(0.1),
    acceleration_scaling_(0.1),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    RCLCPP_INFO(this->get_logger(), "Move to Pose Node has been started.");
    move_group.setStartStateToCurrentState();
    move_group.allowReplanning(true);

    // Set conservative speed
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);

    //start_monitor(); doeesnt do jack shiii
    // Give MoveIt and TF time to initialize
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt and TF to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    pick_service = create_service<std_srvs::srv::Trigger>(
      "move_to_pose",
      std::bind(&MoveToPoseNode::move_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Service for obstacle-avoiding motion (old Trigger version - kept for compatibility)
    planning_service = create_service<std_srvs::srv::Trigger>(
      "move_with_planning",
      std::bind(&MoveToPoseNode::move_with_planning_callback, this, std::placeholders::_1, std::placeholders::_2));

    // New service with full pose control (position + orientation)
    pose_service = create_service<spark_stage_2_ur::srv::MoveToPose>(
      "move_to_pose_full",
      std::bind(&MoveToPoseNode::move_to_pose_full_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Linear move with RPY angles (easier than quaternions)
    linear_rpy_service = create_service<spark_stage_2_ur::srv::MoveLinearRPY>(
      "move_linear_rpy",
      std::bind(&MoveToPoseNode::move_linear_rpy_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Define table collision object
    define_table_collision_object();

    
    RCLCPP_INFO(this->get_logger(), "Node ready - services available:");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose (linear Cartesian, offset-based)");
    RCLCPP_INFO(this->get_logger(), "  - /move_with_planning (obstacle avoidance, hardcoded target)");
    RCLCPP_INFO(this->get_logger(), "  - /move_to_pose_full (full pose control: XYZ + quaternion XYZW)");
    RCLCPP_INFO(this->get_logger(), "  - /move_linear_rpy (linear Cartesian: XYZ + RPY angles)");
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr planning_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveToPose>::SharedPtr pose_service;
  rclcpp::Service<spark_stage_2_ur::srv::MoveLinearRPY>::SharedPtr linear_rpy_service;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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

  

  

  void move_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== LINEAR MOVE START ===");

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
      
      RCLCPP_INFO(get_logger(), "Current pose from TF: (%.3f, %.3f, %.3f)", 
                  current_pose.position.x, current_pose.position.y, current_pose.position.z);
      
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Could not get tool0 transform: %s", ex.what());
      response->success = false;
      response->message = "TF lookup failed";
      return;
    }


    // Target: move in X, Y, Z axes
    // Positive values move: +X (forward), +Y (left), +Z (up)
    // Negative values move: -X (backward), -Y (right), -Z (down)
    double x_offset = 0.332;    // Move in X direction (meters)
    double y_offset = 0.332;    // Move in Y direction (meters)
    double z_offset = -0.332; // Move down 33.2cm (from 0.632 to 0.30)
    
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x = current_pose.position.x + x_offset;
    target_pose.position.y = current_pose.position.y + y_offset;
    target_pose.position.z = current_pose.position.z + z_offset;

    RCLCPP_INFO(get_logger(), "Target: (%.3f, %.3f, %.3f) [Offsets: X=%.3f, Y=%.3f, Z=%.3f]", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                x_offset, y_offset, z_offset);

    // Cartesian path for straight-line motion
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,  // 1cm step
        0.0,   // no jump threshold
        trajectory);

    RCLCPP_INFO(get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);

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
        RCLCPP_INFO(get_logger(), "Trajectory time-parameterized with vel=%.2f, acc=%.2f", 
                    velocity_scaling_, acceleration_scaling_);
      } else {
        RCLCPP_WARN(get_logger(), "Time parameterization failed, using default timing");
      }
      
      //moveit::planning_interface::MoveGroupInterface::Plan plan;
      //plan.trajectory_ = trajectory;
      //move_group.execute(plan);
      
      response->success = true;
      response->message = "Linear path computed (execution disabled).";
      RCLCPP_INFO(get_logger(), "SUCCESS: Linear path ready");
    } else {
      response->success = false;
      response->message = "Could not compute full linear path.";
      RCLCPP_WARN(get_logger(), "FAILED: Only %.1f%% of path computed", fraction * 100.0);
    }

    RCLCPP_INFO(get_logger(), "=== LINEAR MOVE END ===");
  }

  // Service callback for obstacle-avoiding move (using Trigger for now - could use custom msg later)
  void move_with_planning_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Example: Move to a specific position with obstacle avoidance
    // TODO: Replace these hardcoded values with service request parameters
    double target_x = 0.5;   // Change these to desired target
    double target_y = -0.3;
    double target_z = 0.4;
    
    std::string result_msg;
    bool success = move_to_pose_with_planning(target_x, target_y, target_z, result_msg);
    
    response->success = success;
    response->message = result_msg;
  }

  // New callback with full pose control (position + orientation)
  void move_to_pose_full_callback(const std::shared_ptr<spark_stage_2_ur::srv::MoveToPose::Request> request,
                                   std::shared_ptr<spark_stage_2_ur::srv::MoveToPose::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== MOVE TO FULL POSE ===");
    RCLCPP_INFO(get_logger(), "Target position: (%.3f, %.3f, %.3f)", request->x, request->y, request->z);
    RCLCPP_INFO(get_logger(), "Target orientation (quat): (%.3f, %.3f, %.3f, %.3f)", 
                request->quat_x, request->quat_y, request->quat_z, request->quat_w);
    RCLCPP_INFO(get_logger(), "Planning mode: %s", request->use_planning ? "Obstacle avoidance" : "Linear Cartesian");

    // Create target pose from request
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;
    target_pose.orientation.x = request->quat_x;
    target_pose.orientation.y = request->quat_y;
    target_pose.orientation.z = request->quat_z;
    target_pose.orientation.w = request->quat_w;

    bool success = false;
    std::string result_msg;

    if (request->use_planning) {
      // Use obstacle-avoiding planner with shortest path optimization
      move_group.setPoseTarget(target_pose);
      move_group.setPlanningTime(5.0);  // 5 seconds should be enough
      
      // Use RRTConnect - fast and finds shorter paths than default RRT
      move_group.setPlannerId("RRTConnectkConfigDefault");
      
      // Set goal tolerance to avoid unnecessary complex solutions
      move_group.setGoalPositionTolerance(0.001);  // 1mm tolerance
      move_group.setGoalOrientationTolerance(0.01); // ~0.5 degree tolerance
      
      // Try to find shortest path by planning multiple times and picking best
      moveit::planning_interface::MoveGroupInterface::Plan best_plan;
      double best_duration = std::numeric_limits<double>::max();
      bool found_plan = false;
      
      // Try up to 3 planning attempts to find shortest
      for (int attempt = 0; attempt < 3; attempt++) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode plan_result = move_group.plan(plan);
        
        if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
          found_plan = true;
          // Calculate trajectory duration
          double duration = 0.0;
          if (!plan.trajectory_.joint_trajectory.points.empty()) {
            duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                      plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
          }
          
          RCLCPP_INFO(get_logger(), "Attempt %d: Found path with duration %.2fs", attempt + 1, duration);
          
          if (duration < best_duration) {
            best_duration = duration;
            best_plan = plan;
          }
        }
      }
      
      if (found_plan) {
        RCLCPP_INFO(get_logger(), "Best path: %.2fs duration, %zu points",
                    best_duration, best_plan.trajectory_.joint_trajectory.points.size());
        // Uncomment to execute: move_group.execute(best_plan);
        success = true;
        result_msg = "Shortest path found (duration: " + std::to_string(best_duration) + "s, execution disabled)";
      } else {
        success = false;
        result_msg = "Planning failed - no valid path found in 3 attempts";
      }
    } else {
      // Use straight-line Cartesian path
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);
      
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      
      if (fraction > 0.8) {
        // Apply time parameterization for velocity/acceleration scaling
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
        robot_state->setToDefaultValues();
        
        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*robot_state, trajectory);
        
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        time_param.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_);
        rt.getRobotTrajectoryMsg(trajectory);
        
        // Uncomment to execute:
        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // plan.trajectory_ = trajectory;
        // move_group.execute(plan);
        
        success = true;
        result_msg = "Linear Cartesian path computed successfully (execution disabled)";
      } else {
        success = false;
        result_msg = "Cartesian path only " + std::to_string(int(fraction * 100)) + "% achievable";
      }
    }

    response->success = success;
    response->message = result_msg;
  }

  // Linear Cartesian move with RPY angles (easier than quaternions)
  void move_linear_rpy_callback(const std::shared_ptr<spark_stage_2_ur::srv::MoveLinearRPY::Request> request,
                                 std::shared_ptr<spark_stage_2_ur::srv::MoveLinearRPY::Response> response)
  {
    RCLCPP_INFO(get_logger(), "=== LINEAR CARTESIAN MOVE (TF FORMAT) ===");
    RCLCPP_INFO(get_logger(), "Translation xyz: (%.3f, %.3f, %.3f)", 
                request->trans_x, request->trans_y, request->trans_z);
    RCLCPP_INFO(get_logger(), "Rotation xyzw: (%.3f, %.3f, %.3f, %.3f)", 
                request->rot_x, request->rot_y, request->rot_z, request->rot_w);

    // Create target pose in TF format
    geometry_msgs::msg::Pose target_pose;
    // Translation
    target_pose.position.x = request->trans_x;
    target_pose.position.y = request->trans_y;
    target_pose.position.z = request->trans_z;
    // Rotation (quaternion)
    target_pose.orientation.x = request->rot_x;
    target_pose.orientation.y = request->rot_y;
    target_pose.orientation.z = request->rot_z;
    target_pose.orientation.w = request->rot_w;

    // Use straight-line Cartesian path
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
      
      // Uncomment to execute:
      // moveit::planning_interface::MoveGroupInterface::Plan plan;
      // plan.trajectory_ = trajectory;
      // move_group.execute(plan);
      
      response->success = true;
      response->message = "Linear path computed successfully (execution disabled). Fraction: " + 
                         std::to_string(int(fraction * 100)) + "%";
    } else {
      response->success = false;
      response->message = "Cartesian path only " + std::to_string(int(fraction * 100)) + "% achievable - obstacles or joint limits";
    }
  }

  void define_table_collision_object()
  {
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

        if (duration < best_duration) {
          best_duration = duration;
          best_plan = plan;
        }
      }
    }

    if (found_plan) {
      RCLCPP_INFO(get_logger(), "Best path selected: %.2fs, %zu points",
                  best_duration, best_plan.trajectory_.joint_trajectory.points.size());
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
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}