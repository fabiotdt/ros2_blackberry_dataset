#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "move_to_pose_srv/srv/move_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using MoveToPoseSrv = move_to_pose_srv::srv::MoveToPose;

class PoseGoalPlanner : public rclcpp::Node {
 public:
  PoseGoalPlanner()
      : Node("pose_goal_planner_node"),
        move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                    "ur_arm")  // Replace with your MoveIt group
  {
    srv_ = this->create_service<MoveToPoseSrv>(
        "move_to_pose",
        std::bind(&PoseGoalPlanner::poseGoalCallback, this, _1, _2));
    move_group_.setPlannerId("stomp");
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.1);
    move_group_.setMaxAccelerationScalingFactor(0.1);
    // set tolerance
    move_group_.setGoalPositionTolerance(0.04);

    RCLCPP_INFO(this->get_logger(), "PoseGoalPlanner service ready.");
  }

 private:
  rclcpp::Service<MoveToPoseSrv>::SharedPtr srv_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  void poseGoalCallback(const std::shared_ptr<MoveToPoseSrv::Request> request,
                        std::shared_ptr<MoveToPoseSrv::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received pose goal");

    move_group_.setPoseTarget(request->pose,
                              "blackberry");  // Replace link name

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    response->success = success;

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
      move_group_.execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseGoalPlanner>());
  rclcpp::shutdown();
  return 0;
}
