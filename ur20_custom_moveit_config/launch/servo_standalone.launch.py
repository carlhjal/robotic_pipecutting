import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ur_arm"}
    
    servo_params = {
        "moveit_servo": ParameterBuilder("ur20_custom_moveit_config")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }
    
    moveit_config = (
        MoveItConfigsBuilder("custom_ur", package_name="ur20_custom_moveit_config")
        .robot_description(file_path="config/custom_ur.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[
            servo_params,
            update_period,
            planning_group_name,
            {"use_sim_time": False},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            ]
    )

    return LaunchDescription([servo_node])
