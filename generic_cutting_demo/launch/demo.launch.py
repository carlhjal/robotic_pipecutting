import os
import launch
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
        
    package_name = "ur5_cutting_demo"
    package_path = get_package_share_directory(package_name)
    package_moveit_viz_launch_file = os.path.join(package_path, "launch", "ur5_controller_movegroup.launch.py")

    pre_processing_package_name = "reach_planner"
    pre_processing_package_path = get_package_share_directory(pre_processing_package_name)
    pre_processing_pipeline_launchfile = os.path.join(pre_processing_package_path, "launch", "pipeline.launch.py")

    pre_processing = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),
             "launch",
             pre_processing_pipeline_launchfile],
        output="screen",
    )

    robot_moveit_viz = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),
             "launch",
             package_moveit_viz_launch_file],
        output="screen"
    )

    pre_processing_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pre_processing,
            on_exit=robot_moveit_viz
        )
    )

    cartesian_path_demo_package_name = "cartesian_path_waypoints"
    executable_name = "cartesian_path_node"
    cartesian_path_demo = Node(
        package=cartesian_path_demo_package_name,
        executable=executable_name,
        output='screen',
        emulate_tty=True
    )

    monitor_start_output = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=robot_moveit_viz,
            on_stdout=lambda event: ready_for_demo(event, cartesian_path_demo)
        )
    )

    return launch.LaunchDescription([
        pre_processing,
        pre_processing_handler,
        monitor_start_output
    ])


def ready_for_demo(event, cartesian_path_demo):
    output_text = event.text.decode("utf-8")
    if "Ready to take commands" in output_text:
        print("Reach finished. Shutting it down")
        return cartesian_path_demo