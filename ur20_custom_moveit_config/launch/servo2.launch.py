from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
import xacro
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return f.read()

srdf = load_file("ur20_custom_moveit_config", "config/custom_ur.srdf")

def generate_launch_description():
    urdf_file = Command(['xacro ', PathJoinSubstitution([FindPackageShare("ur20_custom_moveit_config"), "config", "custom_ur.urdf.xacro"])])
    srdf_file = PathJoinSubstitution([FindPackageShare("ur20_custom_moveit_config"), "config", "custom_ur.srdf"])
    kinematics_yaml = PathJoinSubstitution([FindPackageShare("ur20_custom_moveit_config"), "config", "kinematics.yaml"])
    servo_yaml = PathJoinSubstitution([FindPackageShare("ur20_custom_moveit_config"), "config", "servo_config2.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("ur20_custom_moveit_config"), "config", "moveit.rviz"])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': urdf_file},
                {'robot_description_semantic': srdf},
                {'robot_description_kinematics': kinematics_yaml}
            ]
        ),

        Node(
            package='moveit_servo',
            executable='servo_node',
            name='moveit_servo',
            output='screen',
            parameters=[
                servo_yaml,
                {'robot_description': urdf_file},
                {'robot_description_semantic': srdf}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[
                {'robot_description': urdf_file},
                {'robot_description_semantic': srdf}
            ],
        )
    ])
