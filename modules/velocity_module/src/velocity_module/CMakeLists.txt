cmake_minimum_required(VERSION 3.8)
project(velocity_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
# Adding the serial package src files
set(serial_SRCS src/serial.cc include/serial/serial.h include/serial/v8stdint.h src/impl/unix.cc)

add_library(serial ${serial_SRCS})

target_link_libraries(serial rt)

# Build
add_executable(velocity_node src/velocity_node.cpp ${serial_SRCS})
ament_target_dependencies(velocity_node
  std_msgs
  ackermann_msgs
  rclcpp
)


# Install
install(TARGETS
  velocity_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()