#include "collision_object_spawner.hpp"

CollisionObjectSpawner::CollisionObjectSpawner(const std::string &frame_id) : frame_id_(frame_id) {}

int CollisionObjectSpawner::spawnObj(const std::string &id,
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
