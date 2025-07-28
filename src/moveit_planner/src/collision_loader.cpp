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

  // TF buffer and listener
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Setup PlanningSceneMonitor
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node, "robot_description", tf_buffer, "planning_scene_monitor");

  if (!planning_scene_monitor->getPlanningScene()) {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured.");
    return EXIT_FAILURE;
  }
  // print planning frame
  // RCLCPP_INFO(LOGGER, "Planning frame: %s",
  //             planning_scene_monitor->getPlanningFrame().c_str());

  // Start monitoring
  // planning_scene_monitor->startSceneMonitor();
  // planning_scene_monitor->startStateMonitor("/joint_states");
  // planning_scene_monitor->startWorldGeometryMonitor();  // Optional: for
  // octomap
  // planning_scene_monitor->setPlanningScenePublishingFrequency(25.0);
  // planning_scene_monitor->startPublishingPlanningScene(
  //     planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
  //     "/planning_scene");

  // Wait for robot state to be ready
  while (!planning_scene_monitor->getStateMonitor()->haveCompleteState()) {
    RCLCPP_INFO(LOGGER, "Waiting for complete joint state...");
    rclcpp::sleep_for(200ms);
  }

  RCLCPP_INFO(LOGGER, "Robot state is ready. Adding collision objects...");

  // Define collision objects
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
  primitives[1].dimensions = {0.24, 1.0, 0.40};
  poses[1].orientation.w = -0.707;
  poses[1].orientation.z = 0.707;
  poses[1].position.x = 0.0;
  poses[1].position.y = -0.255;
  poses[1].position.z = 0.92 - 0.075;

  // Wall
  primitives[2].type = primitives[2].BOX;
  primitives[2].dimensions = {0.05, 1.0, 2.0};
  poses[2].orientation.w = -0.707;
  poses[2].orientation.z = 0.707;
  poses[2].position.x = 0.0;
  poses[2].position.y = -0.375;
  poses[2].position.z = 1.0;

  // Top plate
  primitives[3].type = primitives[3].BOX;
  primitives[3].dimensions = {0.5, 1.0, 0.05};
  poses[3].orientation.w = -0.707;
  poses[3].orientation.z = 0.707;
  poses[3].position.x = 0.0;
  poses[3].position.y = -0.15;
  poses[3].position.z = -0.026;

  // Plug
  primitives[4].type = primitives[4].BOX;
  primitives[4].dimensions = {0.1, 0.12, 0.09};
  poses[4].orientation.w = -0.707;
  poses[4].orientation.z = 0.707;
  poses[4].position.x = 0.44;
  poses[4].position.y = -0.2;
  poses[4].position.z = 0.92 - 0.06;

  // Add objects to planning scene
  planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
  for (size_t i = 0; i < object_ids.size(); ++i) {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = object_ids[i];
    obj.primitives.push_back(primitives[i]);
    obj.primitive_poses.push_back(poses[i]);
    obj.operation = obj.ADD;

    scene->processCollisionObjectMsg(obj);
  }

  // Publish the planning scene
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
      "planning_scene", 1);
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  scene->getPlanningSceneMsg(planning_scene_msg);
  planning_scene_msg.is_diff = true;
  scene_pub->publish(planning_scene_msg);
  RCLCPP_INFO(LOGGER, "Collision objects added to the planning scene.");

  // Keep node alive for a bit to ensure publishing occurs
  rclcpp::sleep_for(2s);
  rclcpp::shutdown();
  return 0;
}
