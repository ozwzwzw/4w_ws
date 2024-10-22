cmake_minimum_required(VERSION 3.8)
project(odom_to_pose)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(odompose_node src/odompose_node.cpp)

# Include directories for header files
target_include_directories(odompose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard
target_compile_features(odompose_node PUBLIC cxx_std_17)

# Link dependencies to the executable
ament_target_dependencies(odompose_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs)

install(TARGETS
  odompose_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
