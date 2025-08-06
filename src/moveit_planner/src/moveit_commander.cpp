#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "json.hpp"

#include <fstream>
#include <filesystem>
#include <random>
#include <vector>
#include <map>
#include <algorithm>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using json = nlohmann::json;
using moveit_msgs::srv::GetPositionIK;
namespace fs = std::filesystem;

class RandomPosePlannerWithIK : public rclcpp::Node
{
public:
  RandomPosePlannerWithIK()
    : Node("random_pose_planner_with_ik"), move_group_(std::shared_ptr<rclcpp::Node>(this), "ur_arm")
  {
    ik_client_ = this->create_client<GetPositionIK>("/compute_ik");
    save_data_client_ = this->create_client<std_srvs::srv::Trigger>("save_berry_data");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

    while (!ik_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for /compute_ik service...");
    }
    while (!save_data_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for save_berry_data service...");
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_data_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::vector<std::pair<geometry_msgs::msg::PoseStamped, std::vector<double>>> poses_;  // pose + joint values

  geometry_msgs::msg::PoseStamped generateRandomPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = this->now();

    static std::random_device rd;
    static std::mt19937 gen(rd());

    //   0.45; 0.0; 0.66
    //   -0.45; 0.6; 0.1

    // std::uniform_real_distribution<double> x_dist(-0.36895, 0.41608);
    // std::uniform_real_distribution<double> y_dist(-0.25725, 0.76453);
    // std::uniform_real_distribution<double> z_dist(0.39237, 0.72183);

    std::uniform_real_distribution<double> x_dist(-0.45, 0.45);
    std::uniform_real_distribution<double> y_dist(-0.6, 0.6);
    std::uniform_real_distribution<double> z_dist(0.1, 0.66);

    std::uniform_real_distribution<double> roll_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> pitch_dist(-M_PI / 2, M_PI / 2);
    std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);

    pose.pose.position.x = x_dist(gen);
    pose.pose.position.y = y_dist(gen);
    pose.pose.position.z = z_dist(gen);

    double roll = roll_dist(gen);
    double pitch = pitch_dist(gen);
    double yaw = yaw_dist(gen);

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    pose.pose.orientation = tf2::toMsg(quat);

    return pose;
  }

  static double jointDistance(const std::vector<double>& a, const std::vector<double>& b)
  {
    double sum_sq = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
      sum_sq += std::pow(a[i] - b[i], 2);
    return std::sqrt(sum_sq);
  }

  void generateAndSavePoses(size_t count)
  {
    rclcpp::sleep_for(std::chrono::seconds(2));
    std::vector<std::string> joint_names = move_group_.getJointNames();
    std::vector<double> reference_joints = move_group_.getCurrentJointValues();
    std::vector<std::pair<geometry_msgs::msg::PoseStamped, std::vector<double>>> valid_poses;

    for (size_t i = 0; i < count; ++i)
    {
      auto pose = generateRandomPose();
      RCLCPP_INFO(this->get_logger(), "Generated pose %zu: [%f, %f, %f] [%f, %f, %f, %f]",
                  i, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                  pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                  pose.pose.orientation.w);
      auto request = std::make_shared<GetPositionIK::Request>();
      request->ik_request.group_name = "ur_arm";
      request->ik_request.pose_stamped = pose;
      request->ik_request.timeout = rclcpp::Duration::from_seconds(0.2);

      auto future = ik_client_->async_send_request(request);
      while (rclcpp::ok())
      {
        auto status = future.wait_for(std::chrono::milliseconds(10));
        if (status == std::future_status::ready)
        {
          break;
        }
      }
      auto response = future.get();
      if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        std::vector<double> joints;
        for (const auto& name : joint_names)
        {
          auto it =
              std::find(response->solution.joint_state.name.begin(), response->solution.joint_state.name.end(), name);
          if (it != response->solution.joint_state.name.end())
          {
            size_t idx = std::distance(response->solution.joint_state.name.begin(), it);
            joints.push_back(response->solution.joint_state.position[idx]);
          }
        }
        valid_poses.emplace_back(pose, joints);
      }
    }
    std::sort(valid_poses.begin(), valid_poses.end(), [&](const auto& a, const auto& b) {
      return jointDistance(a.second, reference_joints) < jointDistance(b.second, reference_joints);
    });

    // Save to JSON
    json poses_json = json::array();
    for (const auto& [pose, joints] : valid_poses)
    {
      json entry;
      entry["position"] = { pose.pose.position.x, pose.pose.position.y, pose.pose.position.z };
      entry["orientation"] = { pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                               pose.pose.orientation.w };
      entry["joints"] = joints;
      poses_json.push_back(entry);
    }
    std::ofstream("poses.json") << poses_json.dump(2);

    // Load into memory
    poses_ = valid_poses;
  }

  void loadPosesAndExecute()
  {
    std::ifstream f("poses.json");
    json poses_json;
    f >> poses_json;

    for (const auto& entry : poses_json)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "base_link";
      pose.pose.position.x = entry["position"][0];
      pose.pose.position.y = entry["position"][1];
      pose.pose.position.z = entry["position"][2];
      pose.pose.orientation.x = entry["orientation"][0];
      pose.pose.orientation.y = entry["orientation"][1];
      pose.pose.orientation.z = entry["orientation"][2];
      pose.pose.orientation.w = entry["orientation"][3];

      std::vector<double> joints = entry["joints"].get<std::vector<double>>();
      poses_.emplace_back(pose, joints);
    }

    executeFromPoses();
  }

  void executeFromPoses()
  {
    size_t resume_index = 0;
    std::ifstream progress_in("progress.json");
    if (progress_in)
    {
      json progress;
      progress_in >> progress;
      resume_index = progress.value("last_index", 0);
      RCLCPP_WARN(this->get_logger(), "Resuming from index %zu", resume_index);
    }

    for (size_t i = resume_index; i < poses_.size(); ++i)
    {
      const auto& [pose, joints] = poses_[i];
      try
      {
        move_group_.setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!move_group_.plan(plan))
        {
          RCLCPP_WARN(this->get_logger(), "Planning failed for pose %zu", i);
          continue;
        }

        pose_pub_->publish(pose);

        auto result = move_group_.execute(plan);
        if (!result)
        {
          RCLCPP_WARN(this->get_logger(), "Execution failed for pose %zu", i);
          continue;
        }

        // Save progress
        std::ofstream("progress.json") << json({ { "last_index", i + 1 } }).dump(2);

        // Call trigger
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = save_data_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          auto res = future.get();
          if (res->success)
          {
            RCLCPP_INFO(this->get_logger(), "Trigger service: %s", res->message.c_str());
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Trigger failed: %s", res->message.c_str());
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception at pose %zu: %s", i, e.what());
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomPosePlannerWithIK>();
  std::thread([&]() { rclcpp::spin(node); }).detach();

  if (fs::exists("poses.json"))
  {
    RCLCPP_INFO(node->get_logger(), "Found existing poses.json. Loading poses...");
    node->loadPosesAndExecute();
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "No poses.json found. Generating poses...");
    node->generateAndSavePoses(5000);  // You can change the count here
    node->executeFromPoses();
  }
  rclcpp::shutdown();
  return 0;
}
