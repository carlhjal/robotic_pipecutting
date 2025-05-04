import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import threading
import time
import os
from scipy.spatial import KDTree
from ament_index_python.packages import get_package_share_directory

package_name = "reach_planner"
package_path = get_package_share_directory(package_name)
pointcloud_path = os.path.join(package_path, "output", "pointcloud.pcd")
mesh_path = os.path.join(package_path, "meshes", "cylinder_lower_away.ply")

def create_circle(radius=0.1, segments=64, z=0.0):
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    points = np.stack([radius * np.cos(angles), radius * np.sin(angles), np.full_like(angles, z)], axis=-1)
    lines = [[i, (i + 1) % segments] for i in range(segments)]
    circle = o3d.geometry.LineSet()
    circle.points = o3d.utility.Vector3dVector(points)
    circle.lines = o3d.utility.Vector2iVector(lines)
    return circle

def generate_circle(radius, z_offset, y_offset, x_plane=0, num_points=50):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = np.full(num_points, x_plane)
    y = radius * np.cos(theta) + y_offset
    z = radius * np.sin(theta) + z_offset
    points = np.vstack((x, y, z)).T
    return points

class App:
    def __init__(self, mesh_path):
        self.auto_orient = True
        self.is_done = False
        self.visible = None
        self.mesh = o3d.io.read_triangle_mesh(mesh_path)
        self.mesh.compute_vertex_normals()
        self.pcd = self.mesh.sample_points_uniformly(number_of_points=2000000)
        self.pcd_downsampled = self.pcd.voxel_down_sample(voxel_size=0.01)
        self.pcd_very_downsampled = self.pcd.voxel_down_sample(voxel_size=0.02)
        # o3d.io.write_point_cloud("whatsthis.pcd", self.pcd, write_ascii=True) to save
        self.visible_pcd = o3d.geometry.PointCloud(self.pcd_downsampled)


        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Trajectory casting", 1024, 768)        
        self.window.set_on_close(self.on_main_window_closing)

        # basic setup
        em = self.window.theme.font_size
        self.layout = gui.Horiz(0, gui.Margins(0.5*em, 0.5*em, 0.5*em, 0.5*em))
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([0, 0, 0, 1])

        # mat
        self.mat_white = rendering.MaterialRecord()
        self.mat_white.shader = "defaultUnlit"
        self.mat_white.point_size = 3
        # self.mat.base_color = ([1, 0, 0, 1])

        self.mat_blue = rendering.MaterialRecord()
        self.mat_blue.shader = "defaultUnlit"
        self.mat_blue.point_size = 5
        self.mat_blue.base_color = ([0, 0, 1, 1])
        
        self.mat_red = rendering.MaterialRecord()
        self.mat_red.shader = "defaultUnlit"
        self.mat_red.point_size = 8
        self.mat_red.base_color = ([1, 0, 0, 1])

        params = {
            "radius": 0.3,
            "z_offset": 0.0,
            "y_offset": 0.0
        }
        self.circle = o3d.geometry.PointCloud()
        self.circle.points = o3d.utility.Vector3dVector(generate_circle(**params))
        self.circle.paint_uniform_color([1, 0, 0])
        self.circle_transform = np.eye(4)

        # add point cloud and circle
        self.scene.scene.add_geometry("pcd", self.pcd_downsampled, self.mat_white)
        bounds = self.pcd.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60, bounds, bounds.get_center())
        # self.circle = create_circle(radius=0.2, z=1.0)
        self.scene.scene.add_geometry("circle", self.circle, self.mat_red)
        
        self.proj_circle = o3d.geometry.PointCloud()
        self.proj_circle.points = o3d.utility.Vector3dVector(generate_circle(**params))
        self.proj_circle.paint_uniform_color([1, 1, 0])
        self.scene.scene.add_geometry("projection", self.proj_circle, self.mat_red)
        
        pole = o3d.geometry.TriangleMesh.create_cylinder(radius=0.01, height=1.0)
        pole.compute_vertex_normals()
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        sphere.compute_vertex_normals()
        self.scene.scene.add_geometry("pole", pole, self.mat_white)
        self.scene.scene.add_geometry("sphere", sphere, self.mat_red)
        self.scene.scene.add_geometry("visible", self.visible_pcd, self.mat_blue)
        
        self.arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=0.005,
            cone_radius=0.01,
            cylinder_height=0.2,
            cone_height=0.03
        )
        self.arrow.compute_vertex_normals()
        self.arrow.paint_uniform_color([0, 1, 0])
        self.scene.scene.add_geometry("arrow", self.arrow, self.mat_white)

        self.panel = gui.Vert(0, gui.Margins(10, 10, 10, 10))
        self.sliders = [self.make_slider("X", 0), self.make_slider("Y", 1), self.make_slider("Z", 2)]

        self.window.add_child(self.scene)
        self.window.add_child(self.panel)

        def on_layout(ctx):
            content_rect = self.window.content_rect
            panel_width = 200
            self.panel.frame = gui.Rect(content_rect.get_right() - panel_width, content_rect.y,
                                        panel_width, content_rect.height)
            self.scene.frame = gui.Rect(content_rect.x, content_rect.y,
                                        content_rect.width - panel_width, content_rect.height)

        self.window.set_on_layout(on_layout)

    def run(self):
        self.app.run()
    
    def on_main_window_closing(self):
        self.is_done = True
        return True

    def make_slider(self, label, idx):
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(-2.0, 2.0)
        slider.set_on_value_changed(lambda val: self.update_circle_position(idx, val))
        self.panel.add_child(gui.Label(label))
        self.panel.add_child(slider)
        return slider
    
    def update_circle_position(self, axis, value):
        self.circle_transform[axis, 3] = value
        self.scene.scene.set_geometry_transform("circle", self.circle_transform)

        R = o3d.geometry.get_rotation_matrix_from_xyz([0, -np.pi/2, 0])  # rotate Z to X
        arrow_transform = self.circle_transform.copy()
        arrow_transform[:3, :3] = self.circle_transform[:3, :3] @ R
        self.scene.scene.set_geometry_transform("arrow", arrow_transform)
        self.fast_update_fov()
        if self.auto_orient == True:
            circle_pos = self.circle_transform[:3, 3]
            self.auto_orientation(circle_pos=circle_pos)
        self.project_on_surface()
        
    def fast_update_fov(self):
        # fast visibility check if only part of the circle projection is on top of the object, the fov should "wrap around" the surface
        view_matrix = self.circle_transform
        pos = view_matrix[:3, 3]
        radius = 2
        _, pt_map = self.pcd_very_downsampled.hidden_point_removal(pos, radius)
        self.visible = self.pcd_very_downsampled.select_by_index(pt_map)

        self.scene.scene.remove_geometry("visible")
        self.scene.scene.add_geometry("visible", self.visible, self.mat_blue)
        self.update_timer = 0

    def auto_orientation(self, circle_pos):
        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd_downsampled)
        # _, idxs, _ = pcd_tree.search_knn_vector_3d(circle_pos, 30)
        _, idxs, _ = pcd_tree.search_radius_vector_3d(circle_pos, radius=0.5)
        normals = np.asarray(self.pcd_downsampled.normals)[idxs]
        avg_normal = np.mean(normals, axis=0)
        avg_normal /= np.linalg.norm(avg_normal)

        x_axis = np.array([1, 0, 0])
        axis = np.cross(x_axis, avg_normal)
        angle = np.arccos(np.dot(x_axis, avg_normal) / (np.linalg.norm(x_axis) * np.linalg.norm(avg_normal)))

        if np.linalg.norm(axis) < 1e-6:
            R = np.eye(3)
        else:
            axis = axis / np.linalg.norm(axis)
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = circle_pos
        self.circle_transform = T

    def project_on_surface(self):
        # get circle rotation and position
        R_circ = self.circle_transform[:3, :3]
        origin = self.circle_transform[:3, 3]

        pcd_np = np.asarray(self.visible.points)
        rel_points = pcd_np - origin
        pcd_in_circle_frame = (R_circ.T @ rel_points.T).T

        # Project into circle's x, y plane
        pcl_2d = pcd_in_circle_frame[:, 1:3]
        circle_local = np.asarray(self.circle.points)
        circle_2d = circle_local[:, 1:3]

        tree = KDTree(pcl_2d)
        _, idx = tree.query(circle_2d)
        projected_points = pcd_np[idx]

        # Visualization
        proj_pcd = o3d.geometry.PointCloud()
        proj_pcd.points = o3d.utility.Vector3dVector(projected_points)
        self.scene.scene.remove_geometry("projection")
        self.scene.scene.add_geometry("projection", proj_pcd, self.mat_red)

def main():
    app = App(mesh_path)
    app.run()
    pass

if __name__ == "__main__":
    main()
