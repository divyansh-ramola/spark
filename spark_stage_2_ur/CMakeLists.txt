cmake_minimum_required(VERSION 3.8)
project(spark_stage_2_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pcb_visualization_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "srv/MoveLinearRPY.srv"
  "srv/MoveLinearQuat.srv"
  "srv/ArduinoCommand.srv"
  "srv/MoveToJoints.srv"
  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# small test executable for quick build checks
add_executable(move_to_pose_node src/move_to_pose_node.cpp)

# Add include directories for headers
target_include_directories(move_to_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(move_to_pose_node 
  rclcpp 
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ur_msgs
  ament_index_cpp
  pcb_visualization_msgs
)

# Link libraries
target_link_libraries(move_to_pose_node yaml-cpp)

# Link the custom service interface to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_pose_node "${cpp_typesupport_target}")



# Arduino Motor Controller node
add_executable(arduino_controller_node src/Arduino_controller_node.cpp)
ament_target_dependencies(arduino_controller_node 
  rclcpp 
  std_srvs
  std_msgs
)

# Link the custom service interface to the Arduino controller
target_link_libraries(arduino_controller_node "${cpp_typesupport_target}")

# Link serial library
target_link_libraries(arduino_controller_node serial)

# TF Pose Saver node
add_executable(tf_pose_saver_node src/tf_pose_saver_node.cpp)
ament_target_dependencies(tf_pose_saver_node 
  rclcpp 
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
)
target_link_libraries(tf_pose_saver_node yaml-cpp)

install(TARGETS
  move_to_pose_node
  arduino_controller_node
  tf_pose_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/connector_pose_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories so launch files and params are available at runtime
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
