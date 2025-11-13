#ifndef CONNECTOR_POSITION_HPP
#define CONNECTOR_POSITION_HPP

#include <vector>
#include <optional>
#include <stdexcept>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace connector_position {

struct MeasuredNode {
    int row; 
    int col;
    double x, y, z;
    double qx, qy, qz, qw;
};

class ConnectorPositionManager {
private:
    std::vector<MeasuredNode> control_nodes_;
    std::vector<int> control_cols_;
    rclcpp::Logger logger_;

    const MeasuredNode* getMeasuredNode(int row, int col) const {
        for (const auto &n : control_nodes_) {
            if (n.row == row && n.col == col) return &n;
        }
        return nullptr;
    }

    std::optional<geometry_msgs::msg::Pose> findMeasuredPoseExact(int row, int col) const {
        for (const auto &n : control_nodes_) {
            if (n.row == row && n.col == col) {
                geometry_msgs::msg::Pose p;
                p.position.x = n.x; p.position.y = n.y; p.position.z = n.z;
                p.orientation.x = n.qx; p.orientation.y = n.qy;
                p.orientation.z = n.qz; p.orientation.w = n.qw;
                return p;
            }
        }
        return std::nullopt;
    }

public:
    ConnectorPositionManager(rclcpp::Logger logger, const std::string& package_name = "spark_stage_2_ur") 
        : logger_(logger) {
        // Default control columns
        control_cols_ = {1, 4, 8, 12, 16};
        
        // Automatically load from YAML on construction
        if (!loadFromYAML(package_name)) {
            RCLCPP_WARN(logger_, "Failed to load YAML, using minimal defaults");
            // Only set minimal defaults if YAML load fails
            control_nodes_ = {
                {1, 1,  0.241, -0.669, 0.288, -0.697, 0.717, -0.008, 0.011},
                {2, 1,  0.243, -0.672, 0.283, -0.697, 0.717, -0.008, 0.011},
                {3, 1,  0.246, -0.674, 0.285, -0.697, 0.717, -0.008, 0.011},
            };
        }
    }

    bool loadFromYAML(const std::string& package_name) {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
            std::string yaml_file = package_share_dir + "/config/connector_poses.yaml";
            
            RCLCPP_INFO(logger_, "Loading connector poses from: %s", yaml_file.c_str());
            
            YAML::Node config = YAML::LoadFile(yaml_file);
            
            if (config["control_nodes"]) {
                control_nodes_.clear();
                for (const auto& node : config["control_nodes"]) {
                    MeasuredNode mn;
                    mn.row = node["row"].as<int>();
                    mn.col = node["col"].as<int>();
                    mn.x = node["x"].as<double>();
                    mn.y = node["y"].as<double>();
                    mn.z = node["z"].as<double>();
                    mn.qx = node["qx"].as<double>();
                    mn.qy = node["qy"].as<double>();
                    mn.qz = node["qz"].as<double>();
                    mn.qw = node["qw"].as<double>();
                    control_nodes_.push_back(mn);
                }
                RCLCPP_INFO(logger_, "✓ Loaded %zu control nodes from YAML", control_nodes_.size());
            }
            
            if (config["control_cols"]) {
                control_cols_.clear();
                for (const auto& col : config["control_cols"]) {
                    control_cols_.push_back(col.as<int>());
                }
                RCLCPP_INFO(logger_, "✓ Loaded %zu control columns", control_cols_.size());
            }
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to load YAML config: %s", e.what());
            return false;
        }
    }

    geometry_msgs::msg::Pose getConnectorPose(int row, int col) {
        if (row < 1 || row > 3) throw std::out_of_range("row must be 1..3");
        if (col < 1 || col > 16) throw std::out_of_range("col must be 1..16");

        // Get base (1,1) for z and orientation - used for ALL poses
        const MeasuredNode* base = getMeasuredNode(1, 1);
        if (!base) throw std::runtime_error("Base pose (1,1) missing.");

        // Check for exact measured pose first (use its x,y but keep base z and orientation)
        auto exact = findMeasuredPoseExact(row, col);
        if (exact.has_value()) {
            RCLCPP_DEBUG(logger_, "Using exact measured pose for (%d,%d) with base z/orientation", row, col);
            geometry_msgs::msg::Pose pose = exact.value();
            // Override z and orientation with base values
            pose.position.z = base->z;
            pose.orientation.x = base->qx;
            pose.orientation.y = base->qy;
            pose.orientation.z = base->qz;
            pose.orientation.w = base->qw;
            return pose;
        }

        // Find column interval for interpolation
        int j_idx = -1;
        for (size_t k = 0; k + 1 < control_cols_.size(); ++k) {
            if (control_cols_[k] <= col && col <= control_cols_[k+1]) { 
                j_idx = static_cast<int>(k); 
                break; 
            }
        }
        if (j_idx == -1) {
            throw std::runtime_error("Requested column outside control column coverage.");
        }
        int c_j = control_cols_[j_idx];
        int c_j1 = control_cols_[j_idx + 1];

        // Determine row interval
        int r0;
        double v; // row fraction
        if (row <= 2) { 
            r0 = 1; 
            v = static_cast<double>(row) - 1.0; 
        } else { 
            r0 = 2; 
            v = static_cast<double>(row) - 2.0; 
        }

        // Get four corner nodes for bilinear interpolation
        const MeasuredNode* n00 = getMeasuredNode(r0,   c_j);
        const MeasuredNode* n10 = getMeasuredNode(r0,   c_j1);
        const MeasuredNode* n01 = getMeasuredNode(r0+1, c_j);
        const MeasuredNode* n11 = getMeasuredNode(r0+1, c_j1);

        if (!(n00 && n10 && n01 && n11)) {
            throw std::runtime_error("Missing control nodes for bilinear interpolation.");
        }

        // Column fraction
        double u;
        if (c_j1 == c_j) u = 0.0;
        else u = (static_cast<double>(col) - static_cast<double>(c_j)) / 
                 static_cast<double>(c_j1 - c_j);

        // Bilinear interpolation for x and y
        double x_interp = (1-u)*(1-v)*n00->x + u*(1-v)*n10->x + 
                         (1-u)*v*n01->x + u*v*n11->x;
        double y_interp = (1-u)*(1-v)*n00->y + u*(1-v)*n10->y + 
                         (1-u)*v*n01->y + u*v*n11->y;

        // base already retrieved at the start of function
        geometry_msgs::msg::Pose out;
        out.position.x = x_interp;
        out.position.y = y_interp;
        out.position.z = base->z;
        out.orientation.x = base->qx;
        out.orientation.y = base->qy;
        out.orientation.z = base->qz;
        out.orientation.w = base->qw;

        RCLCPP_DEBUG(logger_, "Interpolated pose for (%d,%d): [%.3f, %.3f, %.3f]", 
                    row, col, x_interp, y_interp, base->z);
        return out;
    }

    void setMeasuredNode(int row, int col, double x, double y, double z,
                        double qx, double qy, double qz, double qw) {
        for (auto &n : control_nodes_) {
            if (n.row == row && n.col == col) {
                n.x = x; n.y = y; n.z = z;
                n.qx = qx; n.qy = qy; n.qz = qz; n.qw = qw;
                RCLCPP_INFO(logger_, "Updated node (%d,%d)", row, col);
                return;
            }
        }
        control_nodes_.push_back({row, col, x, y, z, qx, qy, qz, qw});
        RCLCPP_INFO(logger_, "Added new node (%d,%d)", row, col);
    }

    void printAllNodes() const {
        RCLCPP_INFO(logger_, "=== Connector Control Nodes ===");
        for (const auto& n : control_nodes_) {
            RCLCPP_INFO(logger_, "{%d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f},", 
                       n.row, n.col, n.x, n.y, n.z, n.qx, n.qy, n.qz, n.qw);
        }
    }
};

} // namespace connector_position

#endif // CONNECTOR_POSITION_HPP
