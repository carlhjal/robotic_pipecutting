#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path");
  const double jump_threshold = 0.0;
  const double eef_step = 0.02;
  moveit_msgs::msg::RobotTrajectory trajectory;

  auto const logger = rclcpp::get_logger("cartesian_path");

  // SingleThreadedExecutor to get current pose of the robot
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

  //x - forward(+) and backward(-)
  //y - left(+) and right(-)
  //z - up(+) and down(-)

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  start_pose.position.z -= 0.2;

  waypoints.push_back(start_pose);

  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

  double radius = 0.15;
  int num_points = 20;
  double center_y = start_pose.position.y;
  double center_z = start_pose.position.z;

  for (int i = 0; i <= num_points; ++i)
  {
    double theta = 2.0 * M_PI * i / num_points; 
    geometry_msgs::msg::Pose waypoint = start_pose;
    waypoint.position.x = center_y + radius * cos(theta);
    waypoint.position.z = center_z + radius * sin(theta);
    waypoints.push_back(waypoint);
  }

  fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

  // Check if complete path is possible and execute the trajectory
  if(fraction == 1){
    move_group_interface.execute(trajectory);
  }
  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}