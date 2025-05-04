from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Launches a self contained demo with simulated time setup
    """
    moveit_config = MoveItConfigsBuilder("custom_ur", package_name="ur_custom_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Declare the use_sim_time argument to use simulated time
    # ld.add_action(
    #     DeclareLaunchArgument(
    #         "use_sim_time",
    #         default_value="true",
    #         description="Use simulated time",
    #     )
    # )

    ld.add_action(
        DeclareLaunchArgument(
            "db",
            default_value="false",
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="By default, we are not in debug mode",
        )
    )

    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py")),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                # {"use_sim_time": True},  # Set use_sim_time for this node
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            remappings=[("/controller_manager/robot_description", "/robot_description")],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/spawn_controllers.launch.py")),
        )
    )

    return ld
