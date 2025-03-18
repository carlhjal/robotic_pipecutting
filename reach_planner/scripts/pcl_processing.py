from pypcd4 import PointCloud
import numpy as np
import os
from matplotlib import pyplot as plt
import random
from scipy.spatial import KDTree
from ament_index_python.packages import get_package_share_directory

class Circle:
    def __init__(self, circle_points, radius, x_plane, z_offset, y_offset):
        self.x: np.ndarray
        self.y: np.ndarray
        self.z: np.ndarray
        self.initialize_circle(circle_points, radius, x_plane, z_offset, y_offset)

    def initialize_circle(self, circle_points, radius, x_plane, z_offset, y_offset):
        theta = np.linspace(0, 2*np.pi, circle_points)
        self.x = np.full(circle_points, x_plane)
        self.y = radius * np.cos(theta) + y_offset
        self.z = radius * np.sin(theta) + z_offset
        

    def update_shape(self, circle_points, radius, x_plane):
        self.initialize_circle(circle_points, radius, x_plane)

def plot_n_samples(num_samples, x, y, z, circle: Circle):
    indices = np.random.choice(len(x), num_samples, replace=False)
    x = x[indices]
    y = y[indices]
    z = z[indices]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, label="cylinder", color="blue")
    ax.scatter(circle.x, circle.y, circle.z, label="circle", color="red")
    ax.scatter(filtered[:,0], filtered[:,1], filtered[:,2], label="circle", color="red")

    # ax.scatter(0,0,0, label="zero", color="magenta")
    ax.view_init(elev=45, azim=180)  # Adjust these values as needed
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    plt.show()

def plot_pointcloud(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, label="cylinder", color="blue")
    ax.scatter(0,0,0, label="zero", color="magenta")
    ax.view_init(elev=45, azim=180)  # Adjust these values as needed
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    plt.show()

def filter_points(pcl_array, path, distance_threshold):
    filtered_points: np.ndarray
    # rough filtering
    pcl_2d = pcl_array[:,1:3]
    pcl_3d = pcl_array[:, :3]
    tree_2d = KDTree(pcl_2d)
    tree_3d = KDTree(pcl_3d)
    distances_2d, indices_2d = tree_2d.query(path[:,-2:])
    distances_3d, indices_3d = tree_3d.query(path)
    closest_2d = pcl_array[indices_2d]
    closest_3d = pcl_array[indices_3d]
    filtered_points = closest_2d
    # filtered_points = (closest_2d + closest_3d) /2
    return filtered_points

package_name = "reach_planner"
package_path = get_package_share_directory(package_name)
pointcloud_path = os.path.join(package_path, "output", "pointcloud.pcd")
pc = PointCloud.from_path(pointcloud_path)
# pc.fields
pcl_array = pc.numpy()
print(pcl_array.shape)

x = pcl_array[:,0]
y = pcl_array[:,1]
z = pcl_array[:,2]

circle = Circle(circle_points=50, radius=0.12, x_plane=0.2, z_offset=0.4, y_offset=-0.2)
circular_path = np.column_stack((circle.x, circle.y, circle.z))
# print(test.shape)
print(circle.x)

mask = (pcl_array[:, 0] < 1)
half_cyl = pcl_array[mask]

filtered = filter_points(half_cyl, circular_path, 0.2)
plot_n_samples(400, x, y, z, circle)
# plot_pointcloud(filtered[:, 0], filtered[:, 1], filtered[:, 2])

# final_array = np.column_stack((filtered[:, 0], filtered[:, 1], filtered[:, 2]))
# print(final_array.shape)
# pcl_back = PointCloud.from_xyz_points(final_array)
# PointCloud.from_a
points = filtered.shape[0]
output_path = os.path.join(package_path, "output", "pointcloud_filtered.pcd")
with open(output_path, "w") as f:
    f.write(f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z normal_x normal_y normal_z curvature
SIZE 4 4 4 4 4 4 4
TYPE F F F F F F F
COUNT 1 1 1 1 1 1 1
WIDTH {points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points}
DATA ascii
""")
    for row in filtered:
        f.write(" ".join(map(str, row)) + "\n")

# # .PCD v0.7 - Point Cloud Data file format
# VERSION 0.7
# FIELDS x y z normal_x normal_y normal_z curvature
# SIZE 4 4 4 4 4 4 4
# TYPE F F F F F F F
# COUNT 1 1 1 1 1 1 1
# WIDTH 75307
# HEIGHT 1
# VIEWPOINT 0 0 0 1 0 0 0
# POINTS 75307
# DATA ascii

# automated wlding robots
# automated welding software stacks
# mathematical concepts, preliminaries
# preliminaries (concepts needed to understand the thesis), related works