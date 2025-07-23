cmake_minimum_required(VERSION 3.8)
project(moveit_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(move_to_pose_srv REQUIRED)

add_executable(collision_loader_node src/collision_loader.cpp)
add_executable(moveit_commander_node src/moveit_commander.cpp)

ament_target_dependencies(collision_loader_node rclcpp moveit_ros_planning_interface moveit_ros_move_group moveit_msgs)
ament_target_dependencies(moveit_commander_node rclcpp moveit_ros_planning_interface moveit_ros_move_group moveit_msgs move_to_pose_srv)

install(TARGETS
  collision_loader_node moveit_commander_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
