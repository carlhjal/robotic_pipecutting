#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <cmath>

std::vector<geometry_msgs::msg::Pose> gen_path(double radius, double cylinder_radius, double height, int num_points) {
  std::vector<geometry_msgs::msg::Pose> waypoints;

  for (int i = 0; i < num_points; i++) {
    double theta = (2 * M_PI * i) / num_points;
    geometry_msgs::msg::Pose pose;
    
  }
}