#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "waypoint_server/srv/get_waypoints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("cartesian_path_logger");
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path");
  rclcpp::executors::SingleThreadedExecutor executor;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  rclcpp::Client<waypoint_server::srv::GetWaypoints>::SharedPtr client =
    node->create_client<waypoint_server::srv::GetWaypoints>("/get_waypoints");
  while (!client->wait_for_service(2s))
  {
      RCLCPP_WARN(logger, "Waiting for service /get_waypoints...");
  }
  auto request = std::make_shared<waypoint_server::srv::GetWaypoints::Request>();
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) 
      == rclcpp::FutureReturnCode::SUCCESS)
    {
      waypoints = result.get()->waypoints;
      RCLCPP_INFO(logger, "Got a response!");
    } else {
      RCLCPP_INFO(logger, "didnt work sorry");
      rclcpp::shutdown();
      return 1;
    }
  
  for (const auto &wp : waypoints) {
    RCLCPP_INFO(logger, "Waypoint: [x: %f, y: %f, z: %f]", wp.position.x, wp.position.y, wp.position.z);
  }  
  
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "ur_arm");

  move_group_interface.setPlanningPipelineId("ompl");
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");  
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setMaxVelocityScalingFactor(0.8);
  move_group_interface.setMaxAccelerationScalingFactor(0.8);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "ur5_base_link";
  target_pose.pose.position.x = waypoints[0].position.x;
  target_pose.pose.position.y = waypoints[0].position.y;
  target_pose.pose.position.z = waypoints[0].position.z;
  target_pose.pose.orientation.x = waypoints[0].orientation.x;
  target_pose.pose.orientation.y = waypoints[0].orientation.y;
  target_pose.pose.orientation.z = waypoints[0].orientation.z;
  target_pose.pose.orientation.w = waypoints[0].orientation.w;
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
  const double jump_threshold = 0.0;
  // consst double eef_step = 0.02;

  for (double eef_step = 0.005; eef_step < 0.2; eef_step = eef_step+0.005) {
    // for (double jump_threshold = 0.0; jump_threshold < 0.1; jump_threshold = jump_threshold+0.01) {
  
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory, true);
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


