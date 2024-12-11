#include <cstdio>
#include <memory>
#include <ros_gz_bridge/bridge_config.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include <gz/msgs/pose_v.pb.h>

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
  config.gz_topic_name = "world/empty/dynamic_pose/info";
  config.ros_type_name = "tf2_msgs/msg/TFMessage";
  // config.ros_type_name = "geometry_msgs/msg/PoseArray";
  config.gz_type_name = "gz.msgs.Pose_V";
  // config.gz_type_name = "gz.msgs.Pose";
  config.is_lazy = lazy_subscription;
 
  bridge_node->add_bridge(config);

  rclcpp::spin(bridge_node);
  return 0;
}

// world/empty/scene/info gz.msgs.Scene updates whenever a new model is added to the scene
// broadcasts info on object position and orientation

// /world/empty/dynamic_pose/info gz.msg.Pose_V links and object position and orientation

// ros2 topic /planning_scene, /monitored_planning_scene has info on collision objects