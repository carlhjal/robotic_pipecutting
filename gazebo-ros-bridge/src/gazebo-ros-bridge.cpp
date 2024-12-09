#include <cstdio>
#include <memory>
#include <ros_gz_bridge/bridge_config.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto bridge_node = std::make_shared<ros_gz_bridge::RosGzBridge>(rclcpp::NodeOptions());
  
  bool lazy_subscription = false;
  bridge_node->declare_parameter<bool>("lazy", false);
  bridge_node->get_parameter("lazy", lazy_subscription);

  ros_gz_bridge::BridgeConfig config;
  config.direction = ros_gz_bridge::BridgeDirection::GZ_TO_ROS;
  config.ros_topic_name = "gz_model_state";
  config.gz_topic_name = "world/empty/pose/info";
  config.ros_type_name = "tf2_msgs/msg/TFMessage";
  config.gz_type_name = "gz.msgs.Pose_V";
  config.is_lazy = lazy_subscription;
 
  bridge_node->add_bridge(config);

  rclcpp::spin(bridge_node);
  return 0;
}
