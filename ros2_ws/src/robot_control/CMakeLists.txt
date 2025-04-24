cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Compile options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories("../../../Shared/Include")


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# Executable
add_executable(robot_control_node src/robot_control_node.cpp)
add_executable(compass_control_node src/compass_control_node.cpp)

# Link dependencies
ament_target_dependencies(robot_control_node
        rclcpp
        geometry_msgs
        robot_interfaces
)

ament_target_dependencies(compass_control_node
        rclcpp
        geometry_msgs
        robot_interfaces
)

# Install target
install(TARGETS
        robot_control_node
        compass_control_node
        DESTINATION lib/${PROJECT_NAME}
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
