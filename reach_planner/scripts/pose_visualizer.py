import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import os

package_name = "reach_planner"
package_path = get_package_share_directory(package_name)
poses_path = os.path.join(package_path, "output", "poses.json")

# Load your JSON
with open(poses_path) as f:
    poses = json.load(f)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for pose in poses:
    pos = np.array([pose["position"]["x"],
                    pose["position"]["y"],
                    pose["position"]["z"]])
    quat = [pose["orientation"]["x"],
            pose["orientation"]["y"],
            pose["orientation"]["z"],
            pose["orientation"]["w"]]
    
    rot = R.from_quat(quat)
    z_axis = rot.apply([0, 0, 0.02])  # short arrow

    ax.quiver(*pos, *z_axis, color='r')

    ax.scatter(*pos, color='k', s=5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Tool Poses with Z-Axis Orientation")
plt.show()
