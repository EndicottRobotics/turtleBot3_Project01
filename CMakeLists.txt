cmake_minimum_required(VERSION 3.8)
project(project_03)

# Compiler flags for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies     
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(endicott_interfaces REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# Create executable for goto_target_node
add_executable(goto_target_node src/goto_target_node.cpp src/endicott_util.cpp)
ament_target_dependencies(goto_target_node rclcpp turtlesim geometry_msgs std_msgs endicott_interfaces)
target_include_directories(goto_target_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create executable for sequence_node
add_executable(sequence_node src/sequence_node.cpp src/endicott_util.cpp)
ament_target_dependencies(sequence_node rclcpp turtlesim geometry_msgs std_msgs endicott_interfaces)
target_include_directories(sequence_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set the required C++ features
target_compile_features(goto_target_node PUBLIC cxx_std_17)
target_compile_features(sequence_node PUBLIC cxx_std_17)

# Install the executables
install(TARGETS goto_target_node  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sequence_node     DESTINATION lib/${PROJECT_NAME})

# Testing and linting setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

 install(DIRECTORY src/launch DESTINATION share/${PROJECT_NAME} )

ament_package()