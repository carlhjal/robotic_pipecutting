import xml.etree.ElementTree as ET
import numpy as np
from geometry_msgs.msg import Pose
# from tf_transformations import quaternion_from_matrix
from transforms3d.quaternions import mat2quat
import json
from ament_index_python.packages import get_package_share_directory
import os


def parse_matrix(matrix_elem):
    """
    Extracts a 4x4 matrix from the XML element.
    """
    items = [float(item.text) for item in matrix_elem.findall('item')]
    if len(items) != 16:
        raise ValueError("Matrix does not contain 16 elements!")
    return np.array(items).reshape(4, 4).T

def matrix_to_pose(matrix):
    """
    Converts a 4x4 transformation matrix to a ROS geometry_msgs/Pose.
    """
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quaternion = mat2quat(matrix[:3, :3])
    pose.orientation.w = quaternion[0]
    pose.orientation.x = quaternion[1]
    pose.orientation.y = quaternion[2]
    pose.orientation.z = quaternion[3]

    return pose

def extract_joint_states(xml_path):
    """
    Extracts the joint states for seeding the robot.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    joint_states = []
    for goal_state_elem in root.findall(".//goal_state"):
        joints = {}
        for joint_elem in goal_state_elem.findall("item"):
            joint_name = joint_elem.find("first").text
            joint_value = float(joint_elem.find("second").text)
            joints[joint_name] = joint_value
        joint_states.append(joints)
    return joint_states

def extract_poses_from_xml(xml_path):
    """
    Extracts transformation matrices as Pose messages.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    unique_poses = {}
    print("Total matrices found:", len(root.findall(".//goal/matrix")))
    for matrix_elem in root.findall(".//goal/matrix"):
        matrix = parse_matrix(matrix_elem)
        pose = matrix_to_pose(matrix)
        pose_tuple = (
            round(pose.position.x, 12),
            round(pose.position.y, 12),
            round(pose.position.z, 12),
            round(pose.orientation.x, 12),
            round(pose.orientation.y, 12),
            round(pose.orientation.z, 12),
            round(pose.orientation.w, 12)
        )
        
        # Overwrite duplicates instead of adding them
        unique_poses[pose_tuple] = pose
    
    print("Total unique poses:", len(unique_poses))
    return list(unique_poses.values())    

def save_poses_json(poses, filename):
    json_data = []

    for pose in poses:
        json_data.append( {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation" : {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        })
    with open(filename, "w") as file:
        json.dump(json_data, file, indent=4)

def save_joint_state_json(joint_state, filename):
    json_data = []
    json_data.append({
        "joint_state": joint_state
    })
    with open(filename, "w") as file:
        json.dump(json_data, file, indent=4)

if __name__ == "__main__":
    package_name = "reach_planner"
    package_path = get_package_share_directory(package_name)
    xml_path = os.path.join(package_path, "output", "results", "ur20", "reach.db.xml")
    poses = extract_poses_from_xml(xml_path)

    joint_states = extract_joint_states(xml_path)
    print(f"joint states len: {len(joint_states)}")
    print(joint_states[0])
    poses_path = os.path.join(package_path, "output", "poses.json")
    save_poses_json(poses, poses_path)
    print(f"Saved poses to {poses_path}")
    
    joint_state_path = os.path.join(package_path, "output", "joint_state.json")
    save_joint_state_json(joint_states[0], joint_state_path)
    print(f"Saved joint state to {joint_state_path}")