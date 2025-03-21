cmake_minimum_required(VERSION 3.8)
project(project_03)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)  

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GotoState.msg"
  DEPENDENCIES std_msgs
)

# Ensure that the generated interface package is added as a dependency for your nodes
# Use project_03_msgs instead of goto_node_msgs
ament_target_dependencies(goto_node rclcpp turtlesim geometry_msgs std_msgs project_03_msgs)
ament_target_dependencies(target_publisher_node rclcpp turtlesim geometry_msgs std_msgs project_03_msgs)

# Create executable for goto_node
add_executable(goto_node src/goto_node.cpp)
ament_target_dependencies(goto_node rclcpp turtlesim geometry_msgs std_msgs project_03_msgs)
target_include_directories(goto_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create executable for target_publisher_node
add_executable(target_publisher_node src/target_publisher_node.cpp)
ament_target_dependencies(target_publisher_node rclcpp turtlesim geometry_msgs std_msgs project_03_msgs)
target_include_directories(target_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set the required C++ features
target_compile_features(goto_node PUBLIC c_std_99 cxx_std_17)

# Install the goto_node executable
install(TARGETS goto_node
  DESTINATION lib/${PROJECT_NAME})

# Testing and linting setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()