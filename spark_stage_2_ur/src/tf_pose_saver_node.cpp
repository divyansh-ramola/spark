// TF Pose Saver Node
// This node reads TF transforms and saves them to a YAML file with auto-incrementing indices.
// 
// Usage:
//   1. Launch the node:
//      ros2 launch spark_stage_2_ur tf_pose_saver.launch.py
//   
//   2. To save a pose, call the service:
//      ros2 service call /save_tf_pose std_srvs/srv/Trigger
//   
//   3. Poses are saved to saved_poses.yaml (configurable) with indices starting from 1
//      Each call increments the index automatically.
//
// Parameters:
//   - yaml_file: Path to output YAML file (default: saved_poses.yaml)
//   - parent_frame: TF parent frame (default: base_link)
//   - child_frame: TF child frame (default: tool0)

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <filesystem>

class TFPoseSaverNode : public rclcpp::Node
{
public:
  TFPoseSaverNode()
  : Node("tf_pose_saver_node"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    this->declare_parameter("yaml_file", "saved_poses.yaml");
    this->declare_parameter("parent_frame", "base_link");
    this->declare_parameter("child_frame", "tool0");
    this->declare_parameter("connector_format", false);  // Save in connector format
    
    yaml_file_ = this->get_parameter("yaml_file").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    connector_format_ = this->get_parameter("connector_format").as_bool();
    
    // Make yaml_file_ absolute path if relative
    if (!yaml_file_.empty() && yaml_file_[0] != '/') {
      yaml_file_ = std::filesystem::current_path().string() + "/" + yaml_file_;
    }
    
    RCLCPP_INFO(this->get_logger(), "TF Pose Saver Node started");
    RCLCPP_INFO(this->get_logger(), "Saving poses to: %s", yaml_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "TF frames: %s -> %s", parent_frame_.c_str(), child_frame_.c_str());
    
    // Load existing index from file if it exists
    current_index_ = loadLastIndex();
    RCLCPP_INFO(this->get_logger(), "Starting from index: %d", current_index_);
    
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_tf_pose",
      std::bind(&TFPoseSaverNode::savePoseCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Service 'save_tf_pose' ready");
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  
  std::string yaml_file_;
  std::string parent_frame_;
  std::string child_frame_;
  bool connector_format_;
  int current_index_;
  
  int loadLastIndex()
  {
    if (!std::filesystem::exists(yaml_file_)) {
      return 1;
    }
    
    try {
      YAML::Node yaml_data = YAML::LoadFile(yaml_file_);
      if (!yaml_data["poses"]) {
        return 1;
      }
      
      int max_index = 0;
      for (const auto& pose_node : yaml_data["poses"]) {
        if (pose_node["index"]) {
          int idx = pose_node["index"].as<int>();
          if (idx > max_index) {
            max_index = idx;
          }
        }
      }
      return max_index + 1;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not load existing YAML file: %s", e.what());
      return 1;
    }
  }
  
  void savePoseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      // Lookup transform
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
        parent_frame_, child_frame_, tf2::TimePointZero);
      
      // Load existing YAML or create new structure
      YAML::Node yaml_data;
      if (std::filesystem::exists(yaml_file_)) {
        yaml_data = YAML::LoadFile(yaml_file_);
      }
      
      if (!yaml_data["poses"]) {
        yaml_data["poses"] = YAML::Node(YAML::NodeType::Sequence);
      }
      
      // Create new pose entry
      YAML::Node pose_entry;
      pose_entry["index"] = current_index_;
      
      YAML::Node position;
      position["x"] = transform_stamped.transform.translation.x;
      position["y"] = transform_stamped.transform.translation.y;
      position["z"] = transform_stamped.transform.translation.z;
      pose_entry["position"] = position;
      
      YAML::Node orientation;
      orientation["x"] = transform_stamped.transform.rotation.x;
      orientation["y"] = transform_stamped.transform.rotation.y;
      orientation["z"] = transform_stamped.transform.rotation.z;
      orientation["w"] = transform_stamped.transform.rotation.w;
      pose_entry["orientation"] = orientation;
      
      pose_entry["frame_id"] = parent_frame_;
      pose_entry["child_frame_id"] = child_frame_;
      
      // Append to poses array
      yaml_data["poses"].push_back(pose_entry);
      
      // Write to file
      std::ofstream fout(yaml_file_);
      fout << yaml_data;
      fout.close();
      
      RCLCPP_INFO(this->get_logger(), 
        "Saved pose %d: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
        current_index_,
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);
      
      // Print in connector format for easy copy-paste
      if (connector_format_) {
        RCLCPP_INFO(this->get_logger(), 
          "Connector format: {%d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f},",
          1,  // row (you can modify this)
          current_index_,  // col
          transform_stamped.transform.translation.x,
          transform_stamped.transform.translation.y,
          transform_stamped.transform.translation.z,
          transform_stamped.transform.rotation.x,
          transform_stamped.transform.rotation.y,
          transform_stamped.transform.rotation.z,
          transform_stamped.transform.rotation.w);
      }
      
      response->success = true;
      response->message = "Pose " + std::to_string(current_index_) + " saved successfully";
      current_index_++;
      
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      response->success = false;
      response->message = std::string("TF lookup failed: ") + ex.what();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save pose: %s", e.what());
      response->success = false;
      response->message = std::string("Failed to save pose: ") + e.what();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFPoseSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
