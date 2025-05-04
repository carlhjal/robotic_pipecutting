#include <cmath>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_servo/moveit_servo/servo.hpp>
#include <moveit_servo/moveit_servo/utils/common.hpp>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <memory>

using namespace std::chrono_literals;

char base_link[] = "base_link";

void send_velocity_command(
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub, 
  const geometry_msgs::msg::Vector3 &direction, 
  rclcpp::Node::SharedPtr node) 
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_link;
  cmd.header.stamp = node->now();
  cmd.twist.linear = direction;
  pub->publish(cmd);
}

bool close_enough(
  const geometry_msgs::msg::Pose &pose, 
  const geometry_msgs::msg::Pose &goal, 
  double threshold_distance=0.005) 
{
  double dx = pose.position.x - goal.position.x;
  double dy = pose.position.y - goal.position.y;
  double dz = pose.position.z - goal.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz) < threshold_distance;
}

void move_with_servo(
  const std::vector<geometry_msgs::msg::Pose> &target_poses,
  moveit::planning_interface::MoveGroupInterface &move_group,
  rclcpp::Node::SharedPtr node)
{
  auto twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/delta_twist_cmds", 10);
  rclcpp::Rate rate(100); // Hz
  for (const auto &target: target_poses) {
    while (rclcpp::ok()) {
      geometry_msgs::msg::Pose current = move_group.getCurrentPose().pose;
      if (close_enough(current, target)) break;

      geometry_msgs::msg::Vector3 dir;
      dir.x = target.position.x - current.position.x;
      dir.y = target.position.y - current.position.y;
      dir.z = target.position.z - current.position.z;

      double mag = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
      if (mag > 1e-4) {
        dir.x *= 0.05 / mag;
        dir.y *= 0.05 / mag;
        dir.z *= 0.05 / mag;
      }

      send_velocity_command(twist_pub, dir, node);
      rate.sleep();
    }
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header.frame_id = base_link;
  stop.header.stamp = node->now();
  twist_pub->publish(stop);
}

std::vector<geometry_msgs::msg::Pose> poses_from_json(const std::string& filename) {
  std::vector<geometry_msgs::msg::Pose> poses;
  std::ifstream file(filename, std::ifstream::binary);

  if (!file.is_open()) {
    std::cerr << "Error opening file" << filename << std::endl;
    return poses;
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string poses_filename = "/home/carl/thesis/thesis_ws/install/reach_planner/share/reach_planner/output/poses.json";
  std::vector<geometry_msgs::msg::Pose> poses = poses_from_json(poses_filename); // load or generate them
  auto const logger = rclcpp::get_logger("servo_logger");
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", false)});
  auto node = std::make_shared<rclcpp::Node>("servo_controller", options);
  // auto const node = std::make_shared<rclcpp::Node>("servo_controller");
  // node->declare_parameter("use_sim_time", false);  // Declare use_sim_time as a parameter
  // node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  bool use_sim_time = node->get_parameter("use_sim_time").as_bool();
  RCLCPP_INFO(logger, "use_sim_time is set to: %s", use_sim_time ? "true" : "false");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  // executor->add_node(shared_from_this());
  // executor->spin();
  // // Get the servo parameters.
  // const std::string param_namespace = "moveit_servo";
  // const std::shared_ptr<const servo::ParamListener> servo_param_listener =
  //     std::make_shared<const servo::ParamListener>(node, param_namespace);
  // const servo::Params servo_params = servo_param_listener->get_params();

  // // The publisher to send trajectory message to the robot controller.
  // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
  //     node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
  //                                                                        rclcpp::SystemDefaultsQoS());
  // const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
  // moveit_servo::createPlanningSceneMonitor(node, servo_params);
  // auto servo = moveit_servo::Servo(node, servo_param_listener, planning_scene_monitor);
  // auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  // const moveit::core::JointModelGroup* joint_model_group =
  //     robot_state->getJointModelGroup(servo_params.move_group_name);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_arm");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  // moveit::planning_interface::MoveGroupInterface::Options options()
  // moveit::planning_interface::MoveGroupInterface::Options options("ur_arm", "robot_description", std::shared_from_this());
  // options.joint_state_topic_ = "/joint_states";
  // options.robot_state_monitor_timeout_ = 0.1; // 100 ms
  // auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), options);
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");  
  move_group.setPlanningTime(15.0);
  move_group.setMaxVelocityScalingFactor(0.8);
  move_group.setMaxAccelerationScalingFactor(0.8);
  // move_group.asyncMove()
  
  // Move the robot to the first pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "ur5_base_link";
  target_pose.pose.position.x = poses[0].position.x;
  target_pose.pose.position.y = poses[0].position.y;
  target_pose.pose.position.z = poses[0].position.z;
  target_pose.pose.orientation.x = poses[0].orientation.x;
  target_pose.pose.orientation.y = poses[0].orientation.y;
  target_pose.pose.orientation.z = poses[0].orientation.z;
  target_pose.pose.orientation.w = poses[0].orientation.w;
  move_group.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful! Executing plan...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();
    return -1;
  }
  auto test = move_group.getEndEffectorLink();
  RCLCPP_INFO(logger, "This is an INFO message!: %s", test.c_str());
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  while (!move_group.getCurrentState(1.0)) {
    RCLCPP_WARN(logger, "Waiting for current robot state...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  geometry_msgs::msg::Pose current = move_group.getCurrentPose().pose;

  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_with_servo(poses, move_group, node);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
