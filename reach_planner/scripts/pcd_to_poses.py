import os, sys
import json
import yaml
import numpy as np
from pypcd4 import PointCloud
from geometry_msgs.msg import Pose
from transforms3d.quaternions import mat2quat
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print("error loading config yaml")
        return None

def load_generic(file_path) -> str:
    with open(file_path) as f:
        return f.read()

def main():
    if not (len(sys.argv) == 3):
        sys.exit()
    config_file_package = sys.argv[1]
    config_file_path = sys.argv[2]
    config_file = os.path.join(config_file_package, config_file_path)
    config = load_yaml(config_file)
    pcd_path = os.path.join(get_package_share_directory(config["pointcloud"]["package"]), config["pointcloud"]["file"])
    pcd = PointCloud.from_path(pcd_path)
    pcd_array = pcd.numpy()
    x = pcd_array[:,0]
    y = pcd_array[:,1]
    z = pcd_array[:,2]
    norm_x = pcd_array[:,3]
    norm_y = pcd_array[:,4]
    norm_z = pcd_array[:,5]

if __name__ == "__main__":
    main()