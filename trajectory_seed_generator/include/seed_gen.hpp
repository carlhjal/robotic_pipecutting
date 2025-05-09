#include "moveit_msgs/msg/robot_trajectory.hpp"
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

std::vector<double> get_viable_seed_state(
    const geometry_msgs::msg::Pose &current_pose,
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    const planning_scene::PlanningScenePtr &scene,
    const moveit::core::RobotModelConstPtr &model,
    const moveit::planning_interface::MoveGroupInterfacePtr &move_group,
    const std::string &group_name,
    double eef_step,
    uint32_t num_trials,
    double ik_timeout
);