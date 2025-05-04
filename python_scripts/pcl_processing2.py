
import numpy.linalg as LA

class InteractiveCircleApp:
    def __init__(self, cloud_path):
        self.is_done = False
        self.cloud_path = cloud_path

        if not os.path.exists(cloud_path):
            raise FileNotFoundError(f"Point cloud not found: {cloud_path}")

        self.cloud = o3d.io.read_point_cloud(cloud_path)
        if len(self.cloud.points) == 0:
            raise ValueError("Loaded point cloud is empty.")

        mesh_path = cloud_path.replace("pointcloud.pcd", "mesh.ply")  # expects mesh in same folder
        if not os.path.exists(mesh_path):
            raise FileNotFoundError(f"Mesh not found: {mesh_path}")
        self.mesh = o3d.io.read_triangle_mesh(mesh_path)
        self.mesh.compute_vertex_normals()

        self.t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)
        self.ray_scene = o3d.t.geometry.RaycastingScene()
        self.mesh_id = self.ray_scene.add_triangles(self.t_mesh)

        self.circle_radius = 0.1
        self.circle_offset = [0.0, 0.0, 0.0]  # x, y, z
        self.circle_resolution = 50

        self.circle = self.create_circle_geometry()
        self.projected_circle = o3d.geometry.PointCloud()
        self.visible_cloud = o3d.geometry.PointCloud(self.cloud)

        self.window = None
        self.scene = None

    def run(self):
        app = gui.Application.instance
        app.initialize()

        self.window = app.create_window("Open3D Interactive Circle", 1024, 768)

        central_layout = gui.Horiz()
        self.window.add_child(central_layout)

        widget3d = gui.SceneWidget()
        widget3d.scene = rendering.Open3DScene(self.window.renderer)
        widget3d.scene.set_background([0, 0, 0, 1])
        self.scene = widget3d.scene

        central_layout.add_child(widget3d)

        bounds = self.cloud.get_axis_aligned_bounding_box()
        self.scene.add_geometry("cloud", self.visible_cloud, self.make_material([0.8, 0.8, 0.8]))
        self.scene.add_geometry("circle", self.circle, self.make_material([1.0, 0.0, 0.0]))
        self.scene.add_geometry("projected", self.projected_circle, self.make_material([0.0, 1.0, 0.0]))

        if bounds.volume() == 0:
            self.scene.camera.look_at([0, 0, 0], [0, 0, -3], [0, 1, 0])
        else:
            self.scene.camera.look_at(
                center=bounds.get_center(),
                eye=bounds.get_center() + [0, 0, -3],
                up=[0, 1, 0]
            )

        self.add_gui_controls(central_layout)
        self.update_loop_thread()
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

        button = gui.Button("Project Circle")
        button.set_on_clicked(self.project_circle_onto_mesh)
        panel.add_fixed(10)
        panel.add_child(button)

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

    def create_circle_geometry(self):
        theta = np.linspace(0, 2 * np.pi, self.circle_resolution)
        x = np.full(self.circle_resolution, self.circle_offset[0])
        y = self.circle_radius * np.cos(theta) + self.circle_offset[1]
        z = self.circle_radius * np.sin(theta) + self.circle_offset[2]
        circle_points = np.vstack((x, y, z)).T

        circle = o3d.geometry.PointCloud()
        circle.points = o3d.utility.Vector3dVector(circle_points)
        circle.paint_uniform_color([1.0, 0.0, 0.0])
        return circle

    def update_circle_geometry(self):
        def do_update():
            theta = np.linspace(0, 2 * np.pi, self.circle_resolution)
            x = np.full(self.circle_resolution, self.circle_offset[0])
            y = self.circle_radius * np.cos(theta) + self.circle_offset[1]
            z = self.circle_radius * np.sin(theta) + self.circle_offset[2]
            circle_points = np.vstack((x, y, z)).T

            self.circle.points = o3d.utility.Vector3dVector(circle_points)
            self.scene.remove_geometry("circle")
            self.scene.add_geometry("circle", self.circle, self.make_material([1.0, 0.0, 0.0]))

        gui.Application.instance.post_to_main_thread(self.window, do_update)

    def project_circle_onto_mesh(self):
        def do_project():
            points = np.asarray(self.circle.points)
            rays = []
            for pt in points:
                rays.append([pt[0], pt[1], pt[2], 0, 0, -1])  # Simple -Z projection
            ray_tensor = o3c.Tensor(rays, dtype=o3c.Dtype.Float32)
            ans = self.ray_scene.cast_rays(ray_tensor)
            projected_points = ans['points'].numpy()
            self.projected_circle.points = o3d.utility.Vector3dVector(projected_points)
            self.scene.remove_geometry("projected")
            self.scene.add_geometry("projected", self.projected_circle, self.make_material([0.0, 1.0, 0.0]))

        gui.Application.instance.post_to_main_thread(self.window, do_project)

    def update_loop_thread(self):
        def loop():
            while True:
                time.sleep(0.05)
                cam = self.scene.camera
                view_matrix = np.asarray(cam.get_view_matrix())
                cam_to_world = LA.inv(view_matrix)
                pos = cam_to_world[:3, 3]
                radius = np.linalg.norm(np.asarray(self.cloud.get_max_bound()) - np.asarray(self.cloud.get_min_bound())) * 100
                _, pt_map = self.cloud.hidden_point_removal(pos, radius)
                visible = self.cloud.select_by_index(pt_map)

                def update():
                    self.visible_cloud.points = visible.points
                    self.scene.remove_geometry("cloud")
                    self.scene.add_geometry("cloud", self.visible_cloud, self.make_material([0.8, 0.8, 0.8]))

                gui.Application.instance.post_to_main_thread(self.window, update)

        threading.Thread(target=loop, daemon=True).start()