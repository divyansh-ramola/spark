// Example usage of connector position manager in move_to_pose_node.cpp
// This shows how to use the connector_manager_ to get poses and move to them

// EXAMPLE 1: Get and move to a single connector position
void example_move_to_single_connector() {
    try {
        // Get pose for row 1, column 4
        geometry_msgs::msg::Pose pose = connector_manager_.getConnectorPose(1, 4);
        
        // Move to that pose
        movelinear(
            pose.position.x,
            pose.position.y, 
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        );
        
        RCLCPP_INFO(get_logger(), "Moved to connector (1,4)");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to get connector pose: %s", e.what());
    }
}

// EXAMPLE 2: Loop through all connectors in a row
void example_loop_through_row() {
    int row = 2;  // Work on row 2
    
    for (int col = 1; col <= 16; ++col) {
        try {
            geometry_msgs::msg::Pose pose = connector_manager_.getConnectorPose(row, col);
            
            RCLCPP_INFO(get_logger(), "Moving to connector (%d,%d)", row, col);
            
            movelinear(
                pose.position.x,
                pose.position.y, 
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            
            // Do something at this position
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            
            // Lift up before moving to next
            joglinear(0.0, 0.0, 0.020);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed at connector (%d,%d): %s", row, col, e.what());
        }
    }
}

// EXAMPLE 3: Visit specific connectors
void example_visit_specific_connectors() {
    std::vector<std::pair<int, int>> connectors_to_visit = {
        {1, 1}, {1, 8}, {1, 16},  // First row corners and middle
        {2, 4}, {2, 12},           // Second row at specific columns
        {3, 1}, {3, 16}            // Third row corners
    };
    
    for (const auto& [row, col] : connectors_to_visit) {
        try {
            geometry_msgs::msg::Pose pose = connector_manager_.getConnectorPose(row, col);
            
            RCLCPP_INFO(get_logger(), "Visiting connector (%d,%d)", row, col);
            
            movelinear(
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w
            );
            
            // Perform task here
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error at (%d,%d): %s", row, col, e.what());
        }
    }
}

// EXAMPLE 4: Update a measured node at runtime (e.g., after manual teach)
void example_update_connector_pose() {
    // After manually moving robot to desired position, read TF and update
    try {
        auto transform = tf_buffer_->lookupTransform(
            "base_link", "tool0", tf2::TimePointZero);
        
        int row = 1, col = 4;  // Position we're updating
        
        connector_manager_.setMeasuredNode(
            row, col,
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        
        RCLCPP_INFO(get_logger(), "Updated connector (%d,%d) with current position", row, col);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
    }
}

// EXAMPLE 5: Print all control nodes for debugging
void example_print_all_nodes() {
    connector_manager_.printAllNodes();
}
