#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <mutex>
#include <string.h>


class MoveToPoseNode : public rclcpp::Node
{
public:
	explicit MoveToPoseNode()
		: Node("move_to_pose_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
		  move_group(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator")
	{
		RCLCPP_INFO(this->get_logger(), "Move to Pose Node has been started.");
		move_group.setStartStateToCurrentState();
		move_group.allowReplanning(true);
		
		// Set looser state freshness tolerance
		move_group.setMaxVelocityScalingFactor(0.1);
		move_group.setMaxAccelerationScalingFactor(0.1);


		start_monitor();
		// Give MoveIt time to connect to joint_states and initialize
		RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt to initialize...");
		rclcpp::sleep_for(std::chrono::seconds(2));
		
		pick_service = create_service<std_srvs::srv::Trigger>("move_to_pose", std::bind(&MoveToPoseNode::move_callback, this, std::placeholders::_1, std::placeholders::_2));
		
		// Define table collision object
		define_table_collision_object();
		
		RCLCPP_INFO(this->get_logger(), "Table collision object added to planning scene.");
		RCLCPP_INFO(this->get_logger(), "Node ready - service 'move_to_pose' available.");
	}



private:
	moveit::planning_interface::MoveGroupInterface move_group;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Joint state tracking
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	std::map<std::string, double> latest_positions_;
	std::mutex js_mutex_;
	bool have_joint_state_ = false;


	void start_monitor(){
		double wait_seconds = 5.0; // try waiting up to 5s for /joint_states
		bool monitor_ok = move_group.startStateMonitor(wait_seconds);
		if (!monitor_ok) {
		RCLCPP_WARN(get_logger(), "startStateMonitor() returned false after %.1f s; joint_states may not be available yet.", wait_seconds);
		} else {
		RCLCPP_INFO(get_logger(), "startStateMonitor() succeeded - current state monitor is ready.");
		}
	}
	void define_table_collision_object(){
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

	void move_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
	{   
		(void)request; // unused
		RCLCPP_INFO(get_logger(), "=== MOTION PLANNING DEBUG START ===");
		
		// // Get current state
		// if (!applyExplicitStartStateFromJointStates()) {
		// 	move_group.setStartStateToCurrentState();
		// }
		// std::string eef = move_group.getEndEffectorLink();
		// RCLCPP_INFO(get_logger(), "End effector link: %s", eef.c_str());
		// RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
		
		// // Log current pose
		// geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose(eef);
		// RCLCPP_INFO(get_logger(), "Current pose (link %s) in frame %s: pos(%.4f, %.4f, %.4f)", 
		// 	eef.c_str(), current.header.frame_id.c_str(),
		// 	current.pose.position.x, current.pose.position.y, current.pose.position.z);
		
	// Set target
	geometry_msgs::msg::PoseStamped target_pose;
	// Use MoveIt's planning frame to avoid TF frame mismatches
	target_pose.header.frame_id = "world";
		target_pose.pose.position.x = 0.4443116;
		target_pose.pose.position.y = -0.595;
		target_pose.pose.position.z = 0.30000;
		target_pose.pose.orientation.x = 0.674; 
		target_pose.pose.orientation.y = 0.736; 
		target_pose.pose.orientation.z = 0.032;
		target_pose.pose.orientation.w = -0.041;

		// Set target with explicit link
		bool result = move_group.setPoseTarget(target_pose);
		RCLCPP_INFO(get_logger(), "setPoseTarget result: %s", result ? "true" : "false");
		
		// move_group.setGoalJointTolerance(0.001);
		// move_group.setGoalOrientationTolerance(0.001);
		// move_group.setGoalPositionTolerance(0.001);
		// move_group.setPlanningTime(5.0);
		
		// Give the current-state monitor a brief moment to receive a fresh state
		//rclcpp::sleep_for(std::chrono::milliseconds(200));
		//move_group.setStartStateToCurrentState();

		//moveit::planning_interface::MoveGroupInterface::Plan plan;
		auto const [success, plan] = [this]{
		  moveit::planning_interface::MoveGroupInterface::Plan msg;
		  auto const ok = static_cast<bool>(this->move_group.plan(msg));
		  return std::make_pair(ok, msg);
		}();
		
		// Execute the plan
			if(success) {
				//move_group.execute(plan);
				response->success = true;
				response->message = "Moved to target pose successfully.";
			} else {
				response->success = false;
				response->message = "Failed to plan to target pose.";
				RCLCPP_ERROR(this->get_logger(), "Planing failed!");
			}
			
		move_group.clearPathConstraints();
		RCLCPP_INFO(get_logger(), "=== MOTION PLANNING DEBUG END ===");
	}

	// Ensure we subscribe to joint states and keep latest by name
	void ensureJointStateSub()
	{
		if (joint_state_sub_) return;
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", rclcpp::QoS(50).best_effort(),
			[this](const sensor_msgs::msg::JointState::SharedPtr msg){
				std::lock_guard<std::mutex> lk(js_mutex_);
				latest_positions_.clear();
				for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
					latest_positions_[msg->name[i]] = msg->position[i];
				}
				have_joint_state_ = !latest_positions_.empty();
			}
		);
	}

	// Build and apply a start state from the latest /joint_states regardless of timestamp freshness
	bool applyExplicitStartStateFromJointStates()
	{
		ensureJointStateSub();
		if (!have_joint_state_) {
			// Give it a brief chance to receive something
			rclcpp::sleep_for(std::chrono::milliseconds(100));
		}
		std::lock_guard<std::mutex> lk(js_mutex_);
		if (!have_joint_state_) {
			return false;
		}
		const auto robot_model = move_group.getRobotModel();
		moveit::core::RobotState start_state(robot_model);
		start_state.setToDefaultValues();
		for (const auto &jn : move_group.getJointNames()) {
			auto it = latest_positions_.find(jn);
			if (it != latest_positions_.end()) {
				start_state.setVariablePosition(jn, it->second);
			}
		}
		move_group.setStartState(start_state);
		return true;
	}
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}