/*
 * Program for testing out moveit cartesian path.
 * Will move the end-effector in a cirular motion
 */

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

    double goal_z = 0.7;
    double goal_y = 0.5;
    double d_z = -(start_pose.position.z - goal_z);
    double d_y = -(start_pose.position.y - goal_y);
    int num_points = 10;
    for (int i = 0; i < num_points; i++) {
        start_pose.position.z += (d_z / num_points);
        start_pose.position.y += (d_y / num_points);
        waypoints.push_back(start_pose);
        RCLCPP_INFO(
    rclcpp::get_logger("debug_logger"), 
        "Pose: Position(x: %f, y: %f, z: %f), Orientation(x: %f, y: %f, z: %f, w: %f)",
        start_pose.position.x, start_pose.position.y, start_pose.position.z,
        start_pose.orientation.x, start_pose.orientation.y, 
        start_pose.orientation.z, start_pose.orientation.w
        );
    }
    
    start_pose.position.z = goal_z;
    start_pose.position.y = goal_y;
    waypoints.push_back(start_pose);

    // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    // RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

    double radius = 0.15;
    num_points = 20;
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

    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

    // Check if complete path is possible and execute the trajectory
    if(fraction == 1){
      move_group_interface.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}