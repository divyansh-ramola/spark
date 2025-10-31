#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <vector>
#include <string>

/**
 * This node remaps joint_states from the robot's actual order to MoveIt's expected order.
 * 
 * Robot publishes: shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan
 * MoveIt expects: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
 */
class JointStateRemapper : public rclcpp::Node
{
public:
    JointStateRemapper() : Node("joint_state_remapper")
    {
        // Subscribe to the original joint_states
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states_raw", 10,
            std::bind(&JointStateRemapper::jointStateCallback, this, std::placeholders::_1));
        
        // Publish remapped joint_states
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // Define the expected order (MoveIt's order)
        expected_order_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
        
        RCLCPP_INFO(this->get_logger(), "Joint State Remapper started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /joint_states_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /joint_states (remapped)");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Create remapped message
        auto remapped_msg = sensor_msgs::msg::JointState();
        remapped_msg.header = msg->header;
        
        // Fix timestamp if it's zero
        if (remapped_msg.header.stamp.sec == 0 && remapped_msg.header.stamp.nanosec == 0) {
            remapped_msg.header.stamp = this->now();
        }
        
        // Build a map of joint name to its values
        std::map<std::string, double> position_map;
        std::map<std::string, double> velocity_map;
        std::map<std::string, double> effort_map;
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            position_map[msg->name[i]] = msg->position[i];
            
            if (i < msg->velocity.size()) {
                velocity_map[msg->name[i]] = msg->velocity[i];
            }
            if (i < msg->effort.size()) {
                effort_map[msg->name[i]] = msg->effort[i];
            }
        }
        
        // Remap to expected order
        remapped_msg.name = expected_order_;
        remapped_msg.position.resize(expected_order_.size());
        remapped_msg.velocity.resize(expected_order_.size());
        remapped_msg.effort.resize(expected_order_.size());
        
        for (size_t i = 0; i < expected_order_.size(); ++i) {
            const std::string& joint_name = expected_order_[i];
            
            if (position_map.find(joint_name) != position_map.end()) {
                remapped_msg.position[i] = position_map[joint_name];
                remapped_msg.velocity[i] = velocity_map[joint_name];
                remapped_msg.effort[i] = effort_map[joint_name];
            } else {
                RCLCPP_WARN_ONCE(this->get_logger(), "Joint %s not found in incoming message", joint_name.c_str());
            }
        }
        
        // Publish remapped message
        pub_->publish(remapped_msg);
        
        if (!logged_once_) {
            RCLCPP_INFO(this->get_logger(), "Successfully remapping joint states");
            RCLCPP_INFO(this->get_logger(), "Input order: %s", vectorToString(msg->name).c_str());
            RCLCPP_INFO(this->get_logger(), "Output order: %s", vectorToString(remapped_msg.name).c_str());
            logged_once_ = true;
        }
    }
    
    std::string vectorToString(const std::vector<std::string>& vec) {
        std::string result = "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            result += vec[i];
            if (i < vec.size() - 1) result += ", ";
        }
        result += "]";
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    std::vector<std::string> expected_order_;
    bool logged_once_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateRemapper>());
    rclcpp::shutdown();
    return 0;
}
