cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_custom_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SetPosition.msg"
  "msg/SetBrake.msg"
)

set(srv_files
  "srv/GetPosition.srv"
)

add_executable(braking_node src/braking_node.cpp)
ament_target_dependencies(braking_node
  dynamixel_sdk
  rclcpp
)

# Install
install(TARGETS
  braking_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
