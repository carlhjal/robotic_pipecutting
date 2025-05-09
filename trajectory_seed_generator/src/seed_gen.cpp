#include "seed_gen.hpp"

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
) {
   auto *joint_model_group = model->getJointModelGroup(group_name);
    if (!joint_model_group) return {};

    for (size_t i = 0; i < num_trials; i++) {
        moveit::core::RobotState state(model);

        if (!state.setFromIK(joint_model_group, current_pose, ik_timeout)) continue;
        if (scene->isStateColliding(state, group_name)) continue;

        std::vector<double> seed;
        state.copyJointGroupPositions(joint_model_group, seed);

        state.update();
        move_group->setStartState(state);

        moveit_msgs::msg::RobotTrajectory traj;
        double fraction = move_group->computeCartesianPath(waypoints, eef_step, traj);
        
        if (fraction == 1.0) return seed;
    }

    return {};
}