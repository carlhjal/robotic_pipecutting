from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # flat YAML with dotted keys
    # servo_yaml = PathJoinSubstitution(
    #     [FindPackageShare("ur20_custom_moveit_config"),
    #      "config", "servo_config_test2.yaml"]
    # )

    servo_yaml = load_yaml("ur20_custom_moveit_config", "config/servo_config_test2.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # MoveIt description etc.
    moveit_config = (
        MoveItConfigsBuilder("custom_ur",
                             package_name="ur20_custom_moveit_config")
        .robot_description(file_path="config/custom_ur.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,          
            # {"use_sim_time": False},
            # {"update_period": 0.01},           # <- add back in
            # {"planning_group_name": "ur_arm"}, # <- add back in
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
