import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.abspath("src/robotic_pipecutting/ur20_custom_moveit_config/config/custom_ur.urdf.xacro")
    controllers_path = os.path.abspath("src/robotic_pipecutting/ur20_custom_moveit_config/config/ros2_controllers.yaml")


    # Load robot_description
    robot_description = {
        "robot_description": os.popen(f"xacro {urdf_path}").read()
    }

    # Load controller YAML into correct format
    with open(controllers_path, "r") as f:
        yaml_dict = yaml.safe_load(f)

    # The key is here 👇 — this unwraps nested structure
    controller_params = yaml_dict["controller_manager"]["ros__parameters"]

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            {"controller_manager": {"ros__parameters": controller_params}}
        ],
        output="screen"
    )

    return LaunchDescription([ros2_control_node])
