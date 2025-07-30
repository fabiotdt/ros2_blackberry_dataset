#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <memory>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vector>

using moveit_msgs::srv::GetPositionIK;

class RandomPosePlannerWithIK : public rclcpp::Node {
 public:
  RandomPosePlannerWithIK()
      : Node("random_pose_planner_with_ik"),
        move_group_(std::shared_ptr<rclcpp::Node>(this), "ur_arm") {
    ik_client_ = this->create_client<GetPositionIK>("/compute_ik");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10);

    while (!ik_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /compute_ik service...");
    }

    RCLCPP_INFO(this->get_logger(), "Random Pose Planner with IK started.");
  }

  geometry_msgs::msg::PoseStamped generateRandomPose() {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = this->now();

    static std::random_device rd;
    static std::mt19937 gen(rd());

    // 0.41608; 0.25725; 0.72183
    // -0.36895; 0.76453; 0.39237
    std::uniform_real_distribution<double> x_dist(-0.36895, 0.41608);
    std::uniform_real_distribution<double> y_dist(-0.25725, 0.76453);
    std::uniform_real_distribution<double> z_dist(0.39237, 0.72183);
    std::uniform_real_distribution<double> roll_dist(90.0, 270.0);
    std::uniform_real_distribution<double> pitch_dist(-150.0, 0.0);
    std::uniform_real_distribution<double> yaw_dist(0.0, 150.0);

    pose.pose.position.x = x_dist(gen);
    pose.pose.position.y = y_dist(gen);
    pose.pose.position.z = z_dist(gen);

    double roll = roll_dist(gen) * M_PI / 180.0;
    double pitch = pitch_dist(gen) * M_PI / 180.0;
    double yaw = yaw_dist(gen) * M_PI / 180.0;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    pose.pose.orientation = tf2::toMsg(quat);

    return pose;
  }

  static double jointDistance(const std::vector<double> &a,
                              const std::vector<double> &b) {
    if (a.size() != b.size()) {
      throw std::runtime_error("Joint vectors must have the same size");
    }
    double sum_sq = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
      double diff = a[i] - b[i];
      sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq);
  }

  void generateAndSortPosesByJointDistance(size_t count = 1000) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::vector<std::string> joint_names = move_group_.getJointNames();
    std::vector<double> reference_joints = move_group_.getCurrentJointValues();

    if (reference_joints.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get current robot joint state.");
      return;
    }

    std::vector<std::pair<geometry_msgs::msg::PoseStamped,
                          moveit_msgs::msg::RobotState>>
        valid_poses;

    for (size_t i = 0; i < count; ++i) {
      auto pose = generateRandomPose();

      auto request = std::make_shared<GetPositionIK::Request>();
      request->ik_request.group_name = "ur_arm";
      request->ik_request.pose_stamped = pose;
      request->ik_request.timeout = rclcpp::Duration::from_seconds(0.2);

      auto future = ik_client_->async_send_request(request);

      while (rclcpp::ok()) {
        auto status = future.wait_for(std::chrono::milliseconds(10));
        if (status == std::future_status::ready) break;
      }

      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "ROS shutdown during IK query.");
        return;
      }

      auto response = future.get();
      if (response->error_code.val ==
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        valid_poses.emplace_back(pose, response->solution);
        RCLCPP_INFO(this->get_logger(), "Pose %zu IK success", i + 1);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Pose %zu IK failed", i + 1);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Generated %zu valid poses. Sorting...",
                valid_poses.size());

    auto extractJoints =
        [&](const moveit_msgs::msg::RobotState &state) -> std::vector<double> {
      std::map<std::string, double> joint_map;
      for (size_t i = 0; i < state.joint_state.name.size(); ++i) {
        joint_map[state.joint_state.name[i]] = state.joint_state.position[i];
      }

      std::vector<double> result;
      for (const auto &name : joint_names) {
        auto it = joint_map.find(name);
        if (it != joint_map.end()) {
          result.push_back(it->second);
        } else {
          throw std::runtime_error("Missing joint " + name +
                                   " in IK solution.");
        }
      }
      return result;
    };

    std::sort(valid_poses.begin(), valid_poses.end(),
              [&](const auto &a, const auto &b) {
                auto joints_a = extractJoints(a.second);
                auto joints_b = extractJoints(b.second);
                return jointDistance(joints_a, reference_joints) <
                       jointDistance(joints_b, reference_joints);
              });

    RCLCPP_INFO(this->get_logger(), "Sorting done. Top 5 poses:");
    for (size_t i = 0; i < std::min(valid_poses.size(), size_t(5)); ++i) {
      const auto &pose = valid_poses[i].first.pose;
      auto joints = extractJoints(valid_poses[i].second);
      double dist = jointDistance(joints, reference_joints);
      RCLCPP_INFO(this->get_logger(),
                  "Pose %zu: pos=(%.3f, %.3f, %.3f), joint dist=%.4f", i + 1,
                  pose.position.x, pose.position.y, pose.position.z, dist);
    }

    for (size_t i = 0; i < valid_poses.size(); ++i) {
      try {
        auto joint_target = extractJoints(valid_poses[i].second);
        move_group_.setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_.plan(plan));

        if (success) {
          RCLCPP_INFO(this->get_logger(),
                      "Planning succeeded for Pose %zu. Executing...", i + 1);

          // 🔽 Publish pose for RViz visualization
          pose_pub_->publish(valid_poses[i].first);

          auto exec_result = move_group_.execute(plan);
          if (!exec_result) {
            RCLCPP_WARN(this->get_logger(),
                        "Execution failed for Pose %zu (error code %d)", i + 1,
                        exec_result.val);
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Planning failed for Pose %zu. Skipping.", i + 1);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));

      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing Pose %zu: %s", i + 1,
                     e.what());
      }
    }
  }

 private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomPosePlannerWithIK>();
  std::thread spin_thread([&]() { rclcpp::spin(node); });
  spin_thread.detach();
  node->generateAndSortPosesByJointDistance(100);
  rclcpp::shutdown();
  return 0;
}
