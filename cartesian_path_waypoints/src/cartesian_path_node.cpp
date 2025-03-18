#include <chrono>
#include <fstream>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <jsoncpp/json/json.h>

std::vector<geometry_msgs::msg::Pose> poses_from_json(const std::string& filename) {
  std::vector<geometry_msgs::msg::Pose> poses;
  std::ifstream file(filename, std::ifstream::binary);

  if (!file.is_open()) {
    std::cerr << "Error opening file" << filename << std::endl;
  }

  Json::Value root;
  file >> root;

  for (const auto& pose_data : root) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_data["position"]["x"].asDouble();
    pose.position.y = pose_data["position"]["y"].asDouble();
    pose.position.z = pose_data["position"]["z"].asDouble();
    pose.orientation.x = pose_data["orientation"]["x"].asDouble();
    pose.orientation.y = pose_data["orientation"]["y"].asDouble();
    pose.orientation.z = pose_data["orientation"]["z"].asDouble();
    pose.orientation.w = pose_data["orientation"]["w"].asDouble();
    
    poses.push_back(pose);
  }
  return poses;
}

using namespace std::chrono_literals;

int main(int argc, char * argv[]) { 
  std::string filename = "/home/carl/thesis/thesis_ws/src/robotic_pipecutting/python_scripts/poses.json";
  std::vector<geometry_msgs::msg::Pose> poses = poses_from_json(filename);

  for (size_t i = 0; i < poses.size(); ++i) {
    std::cout << "Pose " << i << ": Position(" << poses[i].position.x << ", "
              << poses[i].position.y << ", " << poses[i].position.z << ") - Orientation("
              << poses[i].orientation.x << ", " << poses[i].orientation.y << ", "
              << poses[i].orientation.z << ", " << poses[i].orientation.w << ")" << std::endl;
  }
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("cartesian_path_logger");
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path");
  rclcpp::executors::SingleThreadedExecutor executor;
  
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "ur_arm");

  move_group_interface.setPlanningPipelineId("ompl");
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");  
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setMaxVelocityScalingFactor(0.8);
  move_group_interface.setMaxAccelerationScalingFactor(0.8);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "ur5_base_link";
  target_pose.pose.position.x = poses[0].position.x;
  target_pose.pose.position.y = poses[0].position.y;
  target_pose.pose.position.z = poses[0].position.z;
  target_pose.pose.orientation.x = poses[0].orientation.x;
  target_pose.pose.orientation.y = poses[0].orientation.y;
  target_pose.pose.orientation.z = poses[0].orientation.z;
  target_pose.pose.orientation.w = poses[0].orientation.w;
  move_group_interface.setPoseTarget(target_pose);
  
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan...");
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();
    return -1;
  }
  // const double jump_threshold = 0.0;
  // consst double eef_step = 0.02;

  for (double eef_step = 0.005; eef_step < 0.2; eef_step = eef_step+0.005) {
    // for (double jump_threshold = 0.0; jump_threshold < 0.1; jump_threshold = jump_threshold+0.01) {
  
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(poses, eef_step, trajectory, true);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved), eef_step: %f", fraction * 100.0, eef_step);
  
    if(fraction == 1){
      move_group_interface.execute(trajectory);
      break;
      // }
    }
  }
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
