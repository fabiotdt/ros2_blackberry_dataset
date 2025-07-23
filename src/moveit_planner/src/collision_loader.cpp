#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;
#define Z_BASE_LINK 1.79
#define Z_DESK 0.87

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("collision_object_node");

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);
  auto node =
      std::make_shared<rclcpp::Node>("collision_object_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initialize TF buffer and PlanningSceneMonitor
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  // auto planning_scene_monitor =
  //     std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
  //         node, "robot_description", tf_buffer, "planning_scene_monitor");

  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
      "planning_scene", 1);
  while (scene_pub->get_subscription_count() < 1) {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  // if (!planning_scene_monitor->getPlanningScene()) {
  //   RCLCPP_ERROR(LOGGER, "Planning scene not configured.");
  //   return EXIT_FAILURE;
  // }

  // planning_scene_monitor->startStateMonitor("/joint_states");
  // planning_scene_monitor->setPlanningScenePublishingFrequency(25);
  // planning_scene_monitor->startPublishingPlanningScene(
  //     planning_scene_monitor::PlanningSceneMonitor::UPDATE_NONE,
  //     "/planning_scene");

  // planning_scene_monitor->startSceneMonitor();
  // planning_scene_monitor->providePlanningSceneService();

  // Define collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<std::string> object_ids = {"desk", "electric_panel", "wall",
                                         "top_plate", "plug"};
  std::vector<shape_msgs::msg::SolidPrimitive> primitives(5);
  std::vector<geometry_msgs::msg::Pose> poses(5);

  // Desk
  primitives[0].type = primitives[0].BOX;
  primitives[0].dimensions = {1.05, 1.1, Z_BASE_LINK - Z_DESK + 0.01};
  poses[0].orientation.w = -0.707;
  poses[0].orientation.z = 0.707;
  poses[0].position.x = 0.0;
  poses[0].position.y = 0.08;
  poses[0].position.z = Z_DESK + primitives[0].dimensions[2] / 2.0;

  // Electric panel
  primitives[1].type = primitives[1].BOX;
  primitives[1].dimensions = {0.225, 1.0, 0.175};
  poses[1].orientation.w = -0.707;
  poses[1].orientation.z = 0.707;
  // poses[1].position = {0.0, -0.255, 0.92 - 0.075};
  poses[1].position.x = 0.0;
  poses[1].position.y = -0.255;
  poses[1].position.z = 0.92 - 0.075;

  // Wall
  primitives[2].type = primitives[2].BOX;
  primitives[2].dimensions = {0.05, 1.0, 2.0};
  poses[2].orientation.w = -0.707;
  poses[2].orientation.z = 0.707;
  // poses[2].position = {0.0, -0.375, 1.0};
  poses[2].position.x = 0.0;
  poses[2].position.y = -0.375;
  poses[2].position.z = 1.0;

  // Top plate
  primitives[3].type = primitives[3].BOX;
  primitives[3].dimensions = {0.5, 1.0, 0.05};
  poses[3].orientation.w = -0.707;
  poses[3].orientation.z = 0.707;
  // poses[3].position = {0.0, -0.15, -0.026};
  poses[3].position.x = 0.0;
  poses[3].position.y = -0.15;
  poses[3].position.z = -0.026;

  // Plug
  primitives[4].type = primitives[4].BOX;
  primitives[4].dimensions = {0.1, 0.12, 0.09};
  poses[4].orientation.w = -0.707;
  poses[4].orientation.z = 0.707;
  // poses[4].position = {0.44, -0.2, 0.92 - 0.06};
  poses[4].position.x = 0.44;
  poses[4].position.y = -0.2;
  poses[4].position.z = 0.92 - 0.06;

  for (size_t i = 0; i < object_ids.size(); ++i) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = object_ids[i];
    obj.primitives.push_back(primitives[i]);
    obj.primitive_poses.push_back(poses[i]);
    obj.operation = obj.ADD;
    collision_objects.push_back(obj);
  }

  // Publish planning scene update
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world.collision_objects = collision_objects;

  // while (!planning_scene_monitor->getStateMonitor()->haveCompleteState()) {
  //   rclcpp::sleep_for(100ms);
  // }
  RCLCPP_INFO(LOGGER, "Robot state is ready. Publishing planning scene.");

  RCLCPP_WARN(LOGGER, "Publishing planning scene update");
  for (int i = 0; i < 40; ++i) {
    scene_pub->publish(ps);
    rclcpp::sleep_for(100ms);
  }
  rclcpp::shutdown();
  return 0;
}
