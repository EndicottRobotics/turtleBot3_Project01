cmake_minimum_required(VERSION 3.8)
project(turtlebot3_Project01)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler flags for GCC and Clang
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# ---- NODE 1: navigate_to_target_node ----
add_executable(navigate_to_tf_target_node src/navigate_to_tf_target_node.cpp)
ament_target_dependencies(navigate_to_tf_target_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# ---- NODE 2: input_target_tf_node ----
add_executable(input_target_tf_node src/input_target_tf_node.cpp)
ament_target_dependencies(input_target_tf_node rclcpp  tf2 tf2_ros geometry_msgs)

# ---- Install targets ----
install(TARGETS
  navigate_to_tf_target_node
  input_target_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing and linting setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

 install(DIRECTORY src/launch DESTINATION share/${PROJECT_NAME} )

ament_package()