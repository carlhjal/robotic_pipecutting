import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import threading
import time
from ament_index_python.packages import get_package_share_directory
import os
package_name = "reach_planner"
package_path = get_package_share_directory(package_name)
pointcloud_path = os.path.join(package_path, "output", "pointcloud.pcd")



class InteractiveCircleApp:
    def __init__(self, cloud_path):
        self.is_done = False
        self.cloud_path = cloud_path

        if not os.path.exists(cloud_path):
            raise FileNotFoundError(f"Point cloud not found: {cloud_path}")

        self.cloud = o3d.io.read_point_cloud(cloud_path)
        if len(self.cloud.points) == 0:
            raise ValueError("Loaded point cloud is empty.")

        self.circle = None
        self.circle_radius = 0.1
        self.circle_offset = [0.0, 0.0, 0.0]  # x, y, z
        self.circle_resolution = 50

        self.window = None
        self.scene = None

    def run(self):
        app = gui.Application.instance
        app.initialize()

        self.window = app.create_window("Open3D Interactive Circle", 1024, 768)

        # Layout
        central_layout = gui.Horiz()
        self.window.add_child(central_layout)

        # 3D Scene widget
        widget3d = gui.SceneWidget()
        widget3d.scene = rendering.Open3DScene(self.window.renderer)
        widget3d.scene.set_background([0, 0, 0, 1])
        self.scene = widget3d.scene

        # Add scene widget to layout
        central_layout.add_child(widget3d)

        bounds = self.cloud.get_axis_aligned_bounding_box()
        self.scene.add_geometry("cloud", self.cloud, self.make_material([0.8, 0.8, 0.8]))

        if bounds.volume() == 0:
            self.scene.camera.look_at([0, 0, 0], [0, 0, -3], [0, 1, 0])
        else:
            self.scene.camera.look_at(
                center=bounds.get_center(),
                eye=bounds.get_center() + [0, 0, -3],
                up=[0, 1, 0]
            )

        # Add side panel
        self.add_gui_controls(central_layout)
        self.update_circle_geometry()

        gui.Application.instance.run()

    def add_gui_controls(self, layout):
        em = self.window.theme.font_size
        margin = 0.5 * em

        panel = gui.Vert(0.5 * em, gui.Margins(margin))

        radius_slider = gui.Slider(gui.Slider.DOUBLE)
        radius_slider.set_limits(0.01, 1.0)
        radius_slider.double_value = self.circle_radius
        radius_slider.set_on_value_changed(self.on_radius_changed)
        panel.add_child(gui.Label("Circle Radius"))
        panel.add_child(radius_slider)

        labels = ["X Offset", "Y Offset", "Z Offset"]
        for i in range(3):
            slider = gui.Slider(gui.Slider.DOUBLE)
            slider.set_limits(-1.0, 1.0)
            slider.double_value = self.circle_offset[i]
            slider.set_on_value_changed(self.make_offset_callback(i))
            panel.add_child(gui.Label(labels[i]))
            panel.add_child(slider)

        layout.add_fixed(10)
        layout.add_child(panel)

    def make_material(self, color):
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.base_color = color + [1.0]
        mat.point_size = 5.0
        return mat

    def on_radius_changed(self, value):
        self.circle_radius = value
        self.update_circle_geometry()

    def make_offset_callback(self, index):
        def callback(value):
            self.circle_offset[index] = value
            self.update_circle_geometry()
        return callback

    def update_circle_geometry(self):
        def do_update():
            theta = np.linspace(0, 2 * np.pi, self.circle_resolution)
            x = np.full(self.circle_resolution, self.circle_offset[0])
            y = self.circle_radius * np.cos(theta) + self.circle_offset[1]
            z = self.circle_radius * np.sin(theta) + self.circle_offset[2]
            circle_points = np.vstack((x, y, z)).T

            self.circle = o3d.geometry.PointCloud()
            self.circle.points = o3d.utility.Vector3dVector(circle_points)
            self.circle.paint_uniform_color([1.0, 0.0, 0.0])

            self.scene.remove_geometry("circle")
            self.scene.add_geometry("circle", self.circle, self.make_material([1.0, 0.0, 0.0]))

        gui.Application.instance.post_to_main_thread(self.window, do_update)

if __name__ == "__main__":
    cloud_path = pointcloud_path  # replace with your actual path
    InteractiveCircleApp(cloud_path).run()