/**
 * @file move_to_joints_example.cpp
 * @brief Example showing how to send joint angles directly to the robot
 * 
 * This example demonstrates how to use the move_to_joints service to control
 * the robot by specifying joint angles directly, rather than Cartesian poses.
 * 
 * Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
 * All angles are in radians.
 */

#include <rclcpp/rclcpp.hpp>
#include "spark_stage_2_ur/srv/move_to_joints.hpp"
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class JointMoveExample : public rclcpp::Node
{
public:
  JointMoveExample() : Node("joint_move_example")
  {
    client_ = this->create_client<spark_stage_2_ur::srv::MoveToJoints>("move_to_joints");
    
    // Wait for service to be available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for move_to_joints service...");
    }
    
    RCLCPP_INFO(this->get_logger(), "Service is available!");
  }

  bool move_to_joints(const std::vector<double>& joint_angles, const std::string& description = "")
  {
    if (joint_angles.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "Expected 6 joint angles, got %zu", joint_angles.size());
      return false;
    }

    auto request = std::make_shared<spark_stage_2_ur::srv::MoveToJoints::Request>();
    request->joint_positions = joint_angles;
    
    RCLCPP_INFO(this->get_logger(), "Sending joint angles%s", 
                description.empty() ? "" : (": " + description).c_str());
    RCLCPP_INFO(this->get_logger(), "  Angles (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                joint_angles[0], joint_angles[1], joint_angles[2],
                joint_angles[3], joint_angles[4], joint_angles[5]);
    RCLCPP_INFO(this->get_logger(), "  Angles (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                joint_angles[0]*180.0/M_PI, joint_angles[1]*180.0/M_PI, joint_angles[2]*180.0/M_PI,
                joint_angles[3]*180.0/M_PI, joint_angles[4]*180.0/M_PI, joint_angles[5]*180.0/M_PI);
    
    auto future = client_->async_send_request(request);
    
    // Wait for response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✓ Success: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Failed: %s", response->message.c_str());
      }
      return response->success;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

  // Helper function to convert degrees to radians
  static std::vector<double> deg_to_rad(const std::vector<double>& degrees)
  {
    std::vector<double> radians;
    for (double deg : degrees) {
      radians.push_back(deg * M_PI / 180.0);
    }
    return radians;
  }

private:
  rclcpp::Client<spark_stage_2_ur::srv::MoveToJoints>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointMoveExample>();
  
  // Example 1: Home position (all zeros)
  std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  node->move_to_joints(home_position, "Home position");
  
  // Wait a moment
  std::this_thread::sleep_for(2s);
  
  // Example 2: Custom position (example values in radians)
  // Note: Adjust these values based on your robot's workspace and limits
  std::vector<double> custom_position = {
    0.0,           // shoulder_pan: 0 degrees
    -M_PI/4,       // shoulder_lift: -45 degrees
    M_PI/2,        // elbow: 90 degrees
    -M_PI/4,       // wrist_1: -45 degrees
    0.0,           // wrist_2: 0 degrees
    0.0            // wrist_3: 0 degrees
  };
  node->move_to_joints(custom_position, "Custom position");
  
  std::this_thread::sleep_for(2s);
  
  // Example 3: Move with specific joint angles in degrees (converted to radians)
  // Define position in degrees for easier reading
  std::vector<double> position_in_degrees = {0, -90, 90, -90, 0, 0};
  std::vector<double> position_in_radians = JointMoveExample::deg_to_rad(position_in_degrees);
  
  node->move_to_joints(position_in_radians, "Position: [0, -90, 90, -90, 0, 0] degrees");
  
  rclcpp::shutdown();
  return 0;
}
