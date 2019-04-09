#include <jacob_control/ObjectCreator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


std::string ObjectCreator::_name;

ObjectCreator::ObjectCreator(Geometry type, std::string name): PlanningGroup(name) {
    _name = name;
    createObject(type,name);

}

void ObjectCreator::setPrimitive(moveit_msgs::CollisionObject object, Geometry type){
    shape_msgs::SolidPrimitive object_primitive;

    switch (type){
        case BOX:
            object_primitive.type = object_primitive.BOX;
            object_primitive.dimensions.resize(3);
            object_primitive.dimensions[0] = 3.0;
            object_primitive.dimensions[1] = 3.0;
            object_primitive.dimensions[2] = 0.0;
            break;
    }
    object.primitives.push_back(object_primitive);
}

void ObjectCreator::setPosition(moveit_msgs::CollisionObject object){
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position.x = 0.0;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.0;

    object.primitive_poses.push_back(object_pose);
}

void ObjectCreator::createObject(Geometry type, std::string name){
    moveit_msgs::CollisionObject object;
    object.id = name; //ex: "plane"
    object.header.frame_id = getPlanningFrame();

    setPrimitive(object,type);
    setPosition(object);

    object.operation = object.ADD;

    _collision_objects.push_back(object);

    _planning_scene_interface.addCollisionObjects(_collision_objects);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(_move_group->getPlanningFrame());
    visual_tools.trigger();

    

}