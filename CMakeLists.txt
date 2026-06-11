cmake_minimum_required(VERSION 3.8)
project(aubo_ros2_projects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(aubo_dashboard_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ── C++ executables go here ──────────────────────────────────────────────────
# Example:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp trajectory_msgs ...)
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# ── Python scripts ───────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/tcp_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files ─────────────────────────────────────────────────────────────
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
