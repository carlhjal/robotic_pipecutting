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
        
    package_name = "reach_planner"
    package_path = get_package_share_directory(package_name)

    reach_ros_package_name = "reach_ros"
    reach_ros_path = get_package_share_directory(reach_ros_package_name)
    
    reach_custom_setup_path = get_package_share_directory("reach_config")

    setup_launch_file = os.path.join(reach_custom_setup_path, "launch", "setup.launch.py")
    start_launch_file = os.path.join(reach_ros_path, "launch", "start.launch.py")

    results_dir = os.path.join(package_path, "output", "results")

    # Delete directory if it exists
    if os.path.exists(results_dir):
        print(f"Removing existing results directory: {results_dir}")
        shutil.rmtree(results_dir)
    
    robot_rotation = str(math.pi)  # Convert to string for ROS 2 argument passing

    robot_description_file = os.path.join(get_package_share_directory("ur_custom_description"), "urdf", "custom_ur_rotated.urdf.xacro")
    robot_description_semantic_file = os.path.join(get_package_share_directory("ur_custom_moveit_config"), "config", "custom_ur.srdf")
    robot_description_kinematics_file = os.path.join(get_package_share_directory("ur_custom_moveit_config"), "config", "kinematics.yaml")
    robot_description_joint_limits_file = os.path.join(get_package_share_directory("ur_custom_moveit_config"), "config", "joint_limits.yaml")
    config_file = os.path.join(package_path, "config", "reach_config.yaml")
    config_name = "test"
    results_dir = os.path.join(package_path, "output", "results")
    
    # Include setup.launch.py and pass arguments
    setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(setup_launch_file),
        launch_arguments = {
            "robot_description_file": robot_description_file,
            "robot_description_semantic_file": robot_description_semantic_file,
            "use_rviz": "True",  # Set this to True if you want RViz
            "robot_rotation": robot_rotation
        }.items()
    )

    # start_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(start_launch_file),
    #     launch_arguments = {
    #         "robot_description_file": robot_description_file,
    #         "robot_description_semantic_file": robot_description_semantic_file,
    #         "robot_description_kinematics_file": robot_description_kinematics_file,
    #         "robot_description_joint_limits_file": robot_description_joint_limits_file,
    #         "config_file": config_file,
    #         "config_name": config_name,
    #         "results_dir": results_dir
    #     }.items()
    # )

    reach_start_launch = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),
             "launch",
             reach_ros_package_name,
             "start.launch.py",
             f"robot_description_file:={robot_description_file}",
             f"robot_description_semantic_file:={robot_description_semantic_file}",
             f"robot_description_kinematics_file:={robot_description_kinematics_file}",
             f"robot_description_joint_limits_file:={robot_description_joint_limits_file}",
             f"config_file:={config_file}",
             f"config_name:={config_name}",
             f"results_dir:={results_dir}"]
    )

    monitor_start_output = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=reach_start_launch,
            on_stdout=lambda event: shutdown_if_done(event)
        )
    )

    return launch.LaunchDescription([
        setup_launch,
        reach_start_launch,
        monitor_start_output
    ])

def shutdown_if_done(event: RegisterEventHandler):
    output_text = event.text.decode("utf-8")
    if "Press enter to quit" in output_text:
        print("Reach finished. Shutting it down")
        return [EmitEvent(event=launch.events.shutdown())]