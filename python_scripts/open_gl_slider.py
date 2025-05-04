import open3d as o3d
import numpy as np
import dearpygui.dearpygui as dpg
from ament_index_python.packages import get_package_share_directory
import os
import threading

package_name = "reach_planner"
package_path = get_package_share_directory(package_name)
pointcloud_path = os.path.join(package_path, "output", "pointcloud.pcd")
# Settings
POINTCLOUD_PATH = pointcloud_path  # ← change this
DOWNSAMPLED_POINTS = 5000


params = {
    "radius": 0.1,
    "z_offset": 0.0,
    "y_offset": 0.0
}

# Generate a circle based on parameters
def generate_circle(radius, z_offset, y_offset, x_plane=0.2, num_points=50):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = np.full(num_points, x_plane)
    y = radius * np.cos(theta) + y_offset
    z = radius * np.sin(theta) + z_offset
    points = np.vstack((x, y, z)).T
    return points

def run_visualizer(pcd, circle_geom):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Open3D Viewer", width=800, height=600)
    vis.add_geometry(pcd)
    vis.add_geometry(circle_geom)
    
    while True:
        vis.update_geometry(pcd)
        vis.update_geometry(circle_geom)
        vis.poll_events()
        vis.update_renderer()

def update_circle(sender, app_data, user_data):
    new_circle = generate_circle(params["radius"], params["z_offset"], params["y_offset"])
    circle.points = o3d.utility.Vector3dVector(new_circle)

if __name__ == "__main__":
    # Load and downsample
    raw = o3d.io.read_point_cloud(POINTCLOUD_PATH)
    if len(raw.points) > DOWNSAMPLED_POINTS:
        raw = raw.farthest_point_down_sample(DOWNSAMPLED_POINTS)

    # Circle geometry
    circle = o3d.geometry.PointCloud()
    circle.points = o3d.utility.Vector3dVector(generate_circle(**params))
    circle.paint_uniform_color([1, 0, 0])  # red

    # Start visualizer in a background thread
    thread = threading.Thread(target=run_visualizer, args=(raw, circle), daemon=True)
    thread.start()

    # Build DearPyGui UI
    dpg.create_context()
    dpg.create_viewport(title="Sliders", width=400, height=300)

    with dpg.window(label="Circle Controls", width=400, height=300):
        dpg.add_slider_float(label="Radius", default_value=params["radius"], min_value=0.01, max_value=0.5,
                             callback=lambda s, a, u: update_and_store("radius", a))
        dpg.add_slider_float(label="Z Offset", default_value=params["z_offset"], min_value=-1.0, max_value=1.0,
                             callback=lambda s, a, u: update_and_store("z_offset", a))
        dpg.add_slider_float(label="Y Offset", default_value=params["y_offset"], min_value=-1.0, max_value=1.0,
                             callback=lambda s, a, u: update_and_store("y_offset", a))

    def update_and_store(key, val):
        params[key] = val
        update_circle(None, None, None)

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()