cmake_minimum_required(VERSION 3.8)
project(target_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build the node
add_executable(target_node src/target_node.cpp)

# Include directories
target_include_directories(target_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link required libraries
ament_target_dependencies(target_node
rclcpp
std_msgs)

# Set C++ standard
target_compile_features(target_node PUBLIC cxx_std_17)

# Install the executable
install(TARGETS target_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()