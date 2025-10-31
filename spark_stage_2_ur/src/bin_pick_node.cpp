#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //change

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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



        
		
		bool monitor_ok = move_group_interface.startStateMonitor(5.0);
		
		geometry_msgs::msg::PoseStamped current = move_group_interface.getCurrentPose("tool0");

 // RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
// Set a target PoseStamped in the planning frame to avoid TF mismatch
auto const target_pose = [&move_group_interface]{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "world"  ;     //move_group_interface.getPlanningFrame();
  //std::cout << "Planning frame: " << msg.header.frame_id.c_str() << std::endl;
  msg.pose.position.x = 0.4443116;
  msg.pose.position.y = -0.595;
  msg.pose.position.z = 0.30000;
  msg.pose.orientation.x = 0.674; 
  msg.pose.orientation.y = 0.736; 
  msg.pose.orientation.z = 0.032;
  msg.pose.orientation.w = -0.041;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
    //move_group_interface.execute(plan);
} else {
    //RCLCPP_ERROR(logger, "Planing failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}