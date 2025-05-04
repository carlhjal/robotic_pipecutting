import xml.etree.ElementTree as ET
import numpy as np
from geometry_msgs.msg import Pose
# from tf_transformations import quaternion_from_matrix
from transforms3d.quaternions import mat2quat
import json


def parse_matrix(matrix_elem):
    """Extracts a 4x4 matrix from the XML element."""
    items = [float(item.text) for item in matrix_elem.findall('item')]
    if len(items) != 16:
        raise ValueError("Matrix does not contain 16 elements!")
    return np.array(items).reshape(4, 4).T

def matrix_to_pose(matrix):
    """Converts a 4x4 transformation matrix to a ROS geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quaternion = mat2quat(matrix[:3, :3])  # Extract rotation and convert
    pose.orientation.w = quaternion[0]
    pose.orientation.x = quaternion[1]
    pose.orientation.y = quaternion[2]
    pose.orientation.z = quaternion[3]

    return pose

def extract_poses_from_xml(xml_path):
    """Parses an XML file and extracts transformation matrices as Pose messages."""
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
    # poses = []
    # for matrix_elem in root.findall(".//goal/matrix"):
    #     matrix = parse_matrix(matrix_elem)
    #     pose = matrix_to_pose(matrix)
    #     poses.append(pose)
    # return poses

    # poses = []

    # for item_elem in root.findall(".//item"):
    #     score_elem = item_elem.find("score")
    #     if score_elem is not None and float(score_elem.text) > 0:
    #         matrix_elem = item_elem.find("goal/matrix")
    #         if matrix_elem is not None:
    #             matrix = parse_matrix(matrix_elem)
    #             pose = matrix_to_pose(matrix)
    #             poses.append(pose)    
    
    
    # unique_poses = set()
    # poses = []
    
    # for item_elem in root.findall(".//item"):
    #     score_elem = item_elem.find("score")
    #     if score_elem is not None and float(score_elem.text) > 0:
    #         matrix_elem = item_elem.find("goal/matrix")
    #         if matrix_elem is not None:
    #             matrix = parse_matrix(matrix_elem)
    #             pose = matrix_to_pose(matrix)
                
    #             pose_tuple = (
    #                 round(pose.position.x, 6),
    #                 round(pose.position.y, 6),
    #                 round(pose.position.z, 6),
    #                 round(pose.orientation.x, 6),
    #                 round(pose.orientation.y, 6),
    #                 round(pose.orientation.z, 6),
    #                 round(pose.orientation.w, 6)
    #             )
                
    #             if pose_tuple not in unique_poses:
    #                 unique_poses.add(pose_tuple)
    # #                 poses.append(pose)
    # unique_poses = set()
    # poses = []
    
    # for item_elem in root.findall(".//item"):
    #     matrix_elem = item_elem.find("goal/matrix")
    #     if matrix_elem is not None:
    #         matrix = parse_matrix(matrix_elem)
    #         pose = matrix_to_pose(matrix)
            
    #         pose_tuple = (
    #             round(pose.position.x, 6),
    #             round(pose.position.y, 6),
    #             round(pose.position.z, 6),
    #             round(pose.orientation.x, 6),
    #             round(pose.orientation.y, 6),
    #             round(pose.orientation.z, 6),
    #             round(pose.orientation.w, 6)
    #         )
            
    #         if pose_tuple not in unique_poses:
    #             unique_poses.add(pose_tuple)
    #             poses.append(pose)
    
    # return poses
    # """Parses an XML file and extracts transformation matrices, grouping them by class_id."""
    # tree = ET.parse(xml_path)
    # root = tree.getroot()
    
    # class_id_matrices = {}
    
    # for item_elem in root.findall(".//item"):
    #     class_id = item_elem.get("class_id", "unknown")
        
    #     matrix_elem = item_elem.find("goal/matrix")
    #     if matrix_elem is not None:
    #         matrix = parse_matrix(matrix_elem)
    #         pose = matrix_to_pose(matrix)
            
    #         if class_id not in class_id_matrices:
    #             class_id_matrices[class_id] = []
    #         class_id_matrices[class_id].append(pose)
    
    # print("Matrix count per class_id:")
    # for class_id, matrices in class_id_matrices.items():
    #     print(f"Class ID {class_id}: {len(matrices)} matrices")
    
    # return class_id_matrices

# Example usage
def save_json(poses, filename):
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

if __name__ == "__main__":
    xml_file = "/home/carl/thesis/thesis_ws/src/robotic_pipecutting/python_scripts/reach.db.xml"
    poses = extract_poses_from_xml(xml_file)
    
    for i, pose in enumerate(poses):
        print(f"Pose {i}: Position({pose.position.x}, {pose.position.y}, {pose.position.z}) - "
              f"Orientation({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
    
    path = "/home/carl/thesis/thesis_ws/src/robotic_pipecutting/python_scripts/poses.txt"
    json_path = "/home/carl/thesis/thesis_ws/src/robotic_pipecutting/python_scripts/poses.json"
    save_json(poses, json_path)
    with open(path, "w") as f:
        for i, pose in enumerate(poses):
            f.write(f"Pose {i}: Position({pose.position.x}, {pose.position.y}, {pose.position.z}) - "
                f"Orientation({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})\n")
    