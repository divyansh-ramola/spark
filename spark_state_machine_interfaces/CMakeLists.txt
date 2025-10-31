cmake_minimum_required(VERSION 3.8)
project(spark_state_machine_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/GripperStatus.msg"
)

set(srv_files
  "srv/MoveGripper.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
