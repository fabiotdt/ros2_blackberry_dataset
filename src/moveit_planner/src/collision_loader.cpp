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
  primitives[1].dimensions = {0.24, 1.0, 0.35};
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
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  for (size_t i = 0; i < object_ids.size(); ++i) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = object_ids[i];
    collision_object.header.frame_id = "base_link";
    collision_object.primitives.push_back(primitives[i]);
    collision_object.primitive_poses.push_back(poses[i]);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
  }

  // Publish the planning scene
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
      "planning_scene", 1);
  for (int i = 0; i < 10; ++i) {
    rclcpp::sleep_for(100ms);  // Allow time for the publisher to be ready
    scene_pub->publish(planning_scene_msg);
  }
  RCLCPP_INFO(LOGGER, "Collision objects added to the planning scene.");

  // Keep node alive for a bit to ensure publishing occurs
  rclcpp::sleep_for(2s);
  rclcpp::shutdown();
  return 0;
}
