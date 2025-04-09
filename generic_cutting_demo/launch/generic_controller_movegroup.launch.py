import os
import launch
import shutil
import math
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # name of description package here
    robot_description_package_name = "ur20_custom_description"
    # robot_controller_package_name = "ur_custom_control"
    # name of moveit config package here
    robot_moveit_package_name = "ur20_custom_moveit_config"
    cartesian_path_demo_package_name = "cartesian_path_waypoints"

    # robot_controller_package_path = get_package_share_directory(robot_controller_package_name)
    robot_moveit_package_path = get_package_share_directory(robot_moveit_package_name)
    cartesian_path_demo_package_name = get_package_share_directory(cartesian_path_demo_package_name)

    # ros2 launch ur_custom_control start_robot.launch.py use_mock_hardware:=true
    # ros2 launch ur_custom_moveit_config move_group.launch.py 
    # ros2 launch ur_custom_moveit_config moveit_rviz.launch.py 
    
    # start_controller_file = os.path.join(robot_controller_package_path, "launch", "start_robot.launch.py")
    start_movegroup_file = os.path.join(robot_moveit_package_path, "launch", "move_group.launch.py")
    start_moveit_rviz_file = os.path.join(robot_moveit_package_path, "launch", "moveit_rviz.launch.py")

    # start_robot_controller = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(start_controller_file),
    #     launch_arguments = {
    #         "use_mock_hardware": "true",
    #         "launch_rviz": "false"
    #     }.items()
    # )

    start_movegroup_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(start_movegroup_file)
    )

    start_moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(start_moveit_rviz_file)
    )

    return launch.LaunchDescription([
        start_movegroup_controller,
        start_moveit_rviz
    ])
