cmake_minimum_required(VERSION 3.8)
project(vel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build the node
add_executable(velcontrol_node src/velcontrol_node.cpp)

# Include directories
target_include_directories(velcontrol_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link required libraries
ament_target_dependencies(velcontrol_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs)

# Set C++ standard
target_compile_features(velcontrol_node PUBLIC cxx_std_17)

# Install the executable
install(TARGETS velcontrol_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
