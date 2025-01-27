#include "collision_object_spawner.hpp"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

CollisionObjectSpawner::CollisionObjectSpawner(const std::string &frame_id) : frame_id_(frame_id) {}

int CollisionObjectSpawner::spawnPrimitiveObj(const std::string &id,
    const shape_msgs::msg::SolidPrimitive::_type_type &primitive_type,
    const rosidl_runtime_cpp::BoundedVector<double, 3> &dimensions,
    const geometry_msgs::msg::Pose &pose) {
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id_;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive_type;
    primitive.dimensions = dimensions;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);
    return 0;
}

int CollisionObjectSpawner::spawnMeshObj(const std::string &id,
        const std::string &file_name,
        const rosidl_runtime_cpp::BoundedVector<double, 3> &dimensions,
        const geometry_msgs::msg::Pose &pose) {
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id_;
    collision_object.id = id;

    shapes::Mesh *mesh = shapes::createMeshFromResource(file_name);
    if (!mesh) {
        RCLCPP_ERROR(rclcpp::get_logger("CollisionObjectSpawner"), 
        "Failed to load resource: %s", file_name.c_str());
    }
    
}
