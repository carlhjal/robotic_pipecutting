/**
 * @file hello_moveit_v2.cpp
 * @brief A simple ROS 2 and MoveIt 2 program to control a robot arm
 *
 * This program demonstrates how to use ROS 2 and MoveIt 2 to control a robot arm.
 * It sets up a node, creates a MoveGroupInterface for the arm, sets a specific target position,
 * plans a trajectory, and executes the planned motion using the scaled_joint_trajectory_controller.
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
 
  auto const logger = rclcpp::get_logger("hello_moveit");
 
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "ur_arm");

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setMaxVelocityScalingFactor(0.8);
  arm_group_interface.setMaxAccelerationScalingFactor(0.8);

  // arm_group_interface.setWorkspace(-1.0, -1.0, 0.0, 1.0, 1.0, 1.0);
 
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
 
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "ur5_base_link";
  target_pose.pose.position.x = 0.0;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.5;
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose current_pose = arm_group_interface.getCurrentPose().pose;
//   geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  arm_group_interface.setPoseTarget(target_pose);
 
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
 
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan...");
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();
    return -1;
  }
/*
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = target_pose.pose;

  // Circle parameters
  double radius = 0.2;
  int num_points = 20;
  double center_x = start_pose.position.x;
  double center_z = start_pose.position.z + radius;

  for (int i = 0; i <= num_points; ++i)
  {
    double theta = 2.0 * M_PI * i / num_points; 
    geometry_msgs::msg::Pose waypoint = start_pose;
    waypoint.position.x = center_x + radius * cos(theta);
    waypoint.position.z = center_z + radius * sin(theta);
    waypoints.push_back(waypoint);
  }

  // Plan the Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm_group_interface.computeCartesianPath(
    waypoints,
    0.01,
    0.0,        
    trajectory  
  );

  if (fraction < 1.0)
  {
    RCLCPP_WARN(logger, "Could only compute %.2f%% of the Cartesian path", fraction * 100.0);
  }
  else
  {
    RCLCPP_INFO(logger, "Successfully computed Cartesian path");
  }

  if (fraction > 0.0)
  {
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    arm_group_interface.execute(cartesian_plan);
    RCLCPP_INFO(logger, "Executed the Cartesian path.");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to compute Cartesian path.");
  }
*/
  rclcpp::shutdown();
  return 0;
}