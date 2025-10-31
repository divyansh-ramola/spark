#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

class URPosePlanner
{
public:
    URPosePlanner(rclcpp::Node::SharedPtr &node);

private:
    void get_plan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void wait_for_robot_description();
    void define_table_collision_object();
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_pose_service_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

URPosePlanner::URPosePlanner(rclcpp::Node::SharedPtr &node) : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "UR Pose Planner has been started.");
    
    // Wait for robot_description to be available
    wait_for_robot_description();
    
    // Give MoveIt time to initialize
    RCLCPP_INFO(node_->get_logger(), "Waiting for MoveIt to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Create service
    move_to_pose_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/ur_moveit/move_to_pose",
        std::bind(&URPosePlanner::get_plan, this, std::placeholders::_1, std::placeholders::_2));
    
    // Define table collision object
    define_table_collision_object();
    
    RCLCPP_INFO(node_->get_logger(), "Server is up and running.");
}

void URPosePlanner::wait_for_robot_description()
{
    // Check if robot_description parameter is available
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "robot_state_publisher");
    
    RCLCPP_INFO(node_->get_logger(), "Waiting for robot_description parameter...");
    
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for robot_description. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "robot_state_publisher not available, waiting...");
    }
    
    try {
        auto robot_desc = parameters_client->get_parameters({"robot_description"});
        if (!robot_desc.empty() && !robot_desc[0].as_string().empty()) {
            // Set the robot_description parameter on this node
            node_->declare_parameter("robot_description", robot_desc[0].as_string());
            RCLCPP_INFO(node_->get_logger(), "Successfully loaded robot_description from robot_state_publisher");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Could not get robot_description from robot_state_publisher: %s", e.what());
    }
    
    // Load the SRDF from our config file
    std::string srdf_content = R"(<?xml version="1.0" ?>
<robot name="ur10e">
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
  <group name="ur_manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <group_state name="home" group="ur_manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.57"/>
    <joint name="elbow_joint" value="1.57"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="1.57"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
</robot>)";
    
    node_->declare_parameter("robot_description_semantic", srdf_content);
    RCLCPP_INFO(node_->get_logger(), "Loaded robot_description_semantic");
    
    // Load kinematics solver configuration
    node_->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver", 
        "kdl_kinematics_plugin/KDLKinematicsPlugin");
    node_->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_timeout", 0.05);
    node_->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_attempts", 3);
    node_->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution", 0.005);
    RCLCPP_INFO(node_->get_logger(), "Loaded kinematics configuration");
}

void URPosePlanner::define_table_collision_object()
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
    planning_scene_interface_.applyCollisionObjects({table});
    
    RCLCPP_INFO(node_->get_logger(), "Table collision object added to planning scene.");
}

void URPosePlanner::get_plan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // unused
    
    RCLCPP_INFO(node_->get_logger(), "=== MOTION PLANNING DEBUG START ===");
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "ur_manipulator");
    
    // Set velocity and acceleration scaling
    move_group_interface.setMaxVelocityScalingFactor(0.4);
    move_group_interface.setMaxAccelerationScalingFactor(0.4);
    
    std::string eef = move_group_interface.getEndEffectorLink();
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", eef.c_str());
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    
    // Log MoveIt's expected joint order
    RCLCPP_INFO(node_->get_logger(), "MoveIt joint order:");
    for (size_t i = 0; i < move_group_interface.getJointNames().size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "  [%zu] %s", i, move_group_interface.getJointNames()[i].c_str());
    }
    
    // Set starting state from known joint positions
    // joint_states order: shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan
    // MoveIt order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
    std::map<std::string, double> start_joint_values;
    start_joint_values["shoulder_pan_joint"] = 2.0964;   // From joint_states[5]
    start_joint_values["shoulder_lift_joint"] = -1.8085; // From joint_states[0]
    start_joint_values["elbow_joint"] = -0.5256;         // From joint_states[1]
    start_joint_values["wrist_1_joint"] = -2.4547;       // From joint_states[2]
    start_joint_values["wrist_2_joint"] = 1.6134;        // From joint_states[3]
    start_joint_values["wrist_3_joint"] = -1.0672;       // From joint_states[4]
    
    // Create a new robot state from the robot model
    moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(move_group_interface.getRobotModel()));
    start_state->setToDefaultValues();
    start_state->setVariablePositions(start_joint_values);
    move_group_interface.setStartState(*start_state);
    
    RCLCPP_INFO(node_->get_logger(), "Set starting state manually (MoveIt order):");
    for (const auto& joint_name : move_group_interface.getJointNames()) {
        RCLCPP_INFO(node_->get_logger(), "  %s: %.4f rad", 
            joint_name.c_str(), start_joint_values[joint_name]);
    }
    
    // Set target pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.pose.position.x = -0.31236;
    target_pose.pose.position.y = 0.497459;
    target_pose.pose.position.z = 0.92063;
    target_pose.pose.orientation.x = 0.756073;
    target_pose.pose.orientation.y = 0.422992;
    target_pose.pose.orientation.z = -0.49493;
    target_pose.pose.orientation.w = 0.06685;
    
    // Set target with explicit link
    bool result = move_group_interface.setPoseTarget(target_pose, eef);
    RCLCPP_INFO(node_->get_logger(), "setPoseTarget result: %s", result ? "true" : "false");
    
    // Set planning parameters
    move_group_interface.setGoalJointTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.001);
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setPlanningTime(5.0);
    
    // Plan
    auto const [success, plan] = [&move_group_interface]() {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    if (success) {
        // Log the joint angles that will be executed WITH NAMES
        RCLCPP_INFO(node_->get_logger(), "Plan successful! Joint trajectory:");
        RCLCPP_INFO(node_->get_logger(), "Trajectory joint order:");
        for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "  [%zu] %s", i,
                plan.trajectory_.joint_trajectory.joint_names[i].c_str());
        }
        
        RCLCPP_INFO(node_->get_logger(), "Final point joint values:");
        auto final_point = plan.trajectory_.joint_trajectory.points.back();
        for (size_t i = 0; i < final_point.positions.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "  %s: %.4f rad",
                plan.trajectory_.joint_trajectory.joint_names[i].c_str(),
                final_point.positions[i]);
        }
        
        // NOT executing - only planning for debugging
        RCLCPP_INFO(node_->get_logger(), "Planning successful (execution disabled for debugging)");
        response->success = true;
        response->message = "Planning succeeded (not executed).";
    } else {
        response->success = false;
        response->message = "Failed to plan to target pose.";
        RCLCPP_WARN(node_->get_logger(), "Failed to plan!");
    }
    
    move_group_interface.clearPathConstraints();
    RCLCPP_INFO(node_->get_logger(), "=== MOTION PLANNING DEBUG END ===");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto main_node = std::make_shared<rclcpp::Node>("ur_pose_planner", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    URPosePlanner ur_pose_planner_object(main_node);
    executor.add_node(main_node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
