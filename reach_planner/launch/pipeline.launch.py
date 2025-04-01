import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable

def generate_launch_description():
    package_name = "reach_planner"
    package_path = get_package_share_directory(package_name)

    workspace_install_path = os.getenv("COLCON_PREFIX_PATH")
    workspace_base_path = os.path.join(workspace_install_path, "..")

    # Step 1: Run pcl_mesh_sampling
    mesh_path = os.path.join(package_path, "meshes", "cylinder_lower_away.ply")
    output_pcd = os.path.join(package_path, "output", "pointcloud.pcd")
    pcl_mesh_sampling = ExecuteProcess(
        cmd=["pcl_mesh_sampling", 
             mesh_path, 
             output_pcd,
             "-leaf_size", "0.001",
             "-n_samples", "1500000",
             "-write_normals", "true",
             "-use_triangle_normal", "true",
             "-no_vis_result"
            ],
        output="screen"
    )

    # Step 2: Run python pcl-processing point-filtering script
    pcl_processing_path = os.path.join(package_path, "scripts", "pcl_processing.py")
    venv_path = os.path.join(workspace_base_path, ".venv", "bin", "python3")
    pcl_processing = ExecuteProcess(
        cmd=[venv_path, pcl_processing_path, "no-gui"],
        output="screen"
    )

    pcl_processing_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pcl_mesh_sampling,
            on_exit=pcl_processing
        )
    )

    # Step 3: Run reach analysis
    reach_analysis = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"),
             "launch",
             package_name,
             "reach_analysis.launch.py"
             ],
        output="screen"
    )

    reach_processing_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pcl_processing,
            on_exit=reach_analysis
        )
    )

    # Step 4: Run reach database pose extractor script
    db_extractor_path = os.path.join(package_path, "scripts", "db_extractor.py")
    db_processing = ExecuteProcess(
        cmd=[venv_path, db_extractor_path],
        output="screen"
    )

    db_processing_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=reach_analysis,
            on_exit=db_processing
        )
    )

    return launch.LaunchDescription([
        pcl_mesh_sampling,
        pcl_processing_handler,
        reach_processing_handler,
        db_processing_handler
    ])