import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "reach_config"  # Replace with your actual package name
    mesh_file = "config/cylinder_lower_away.ply"
    mesh_path = os.path.join(get_package_share_directory(package_name), mesh_file)
    print(mesh_path)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="object_spawner",  # Replace with your actual package name
            executable="mesh_spawner",
            name="mesh_spawner",
            output="screen",
            parameters=[{"mesh_file": mesh_path}]
        )
    ])