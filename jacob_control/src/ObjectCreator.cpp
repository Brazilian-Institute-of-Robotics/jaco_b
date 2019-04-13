#include <jacob_control/ObjectCreator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


std::string ObjectCreator::_name = "object_handle";

ObjectCreator::ObjectCreator(Geometry type, std::string name, ros::NodeHandle node): PlanningGroup(_name) {
    _name = "object_handle"; //same from srdf
    createObject(type,name,node);

}

void ObjectCreator::setPrimitive(moveit_msgs::CollisionObject& object, Geometry type){
    shape_msgs::SolidPrimitive object_primitive;

    switch (type){
        case BOX:
            ROS_INFO_STREAM("is a box");
            object_primitive.type = object_primitive.BOX;
            object_primitive.dimensions.resize(3);
            object_primitive.dimensions[0] = 3.0;
            object_primitive.dimensions[1] = 3.0;
            object_primitive.dimensions[2] = 0.0;
            break;
    }
    object.primitives.push_back(object_primitive);
    if (object.primitives.empty()){
        ROS_INFO_STREAM("IS EMPTY");
    }
    else{
        ROS_INFO_STREAM("NOT EMPTY");
    }
}

void ObjectCreator::setPosition(moveit_msgs::CollisionObject& object){
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position.x = 0.0;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.0;

    object.primitive_poses.push_back(object_pose);
}

void ObjectCreator::createObject(Geometry type, std::string name, ros::NodeHandle node){
    _collision_object.id = name; //ex: "plane"
    _collision_object.header.frame_id = getPlanningFrame();

    setPrimitive(_collision_object,type);

    if (_collision_object.primitives.empty()){
        ROS_INFO_STREAM("IS EMPTY");
    }
    else{
        ROS_INFO_STREAM("NOT EMPTY");
    }
    
    setPosition(_collision_object);

    _collision_object.operation = _collision_object.ADD;

    //_collision_objects.push_back(object);

    //_planning_scene_interface.addCollisionObjects(_collision_objects);

    _planning_scene.world.collision_objects.push_back(_collision_object);
    _planning_scene.is_diff = true;

    planning_scene_diff_publisher = node.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    ros::WallDuration sleep_t(0.5);
     while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    planning_scene_diff_publisher.publish(_planning_scene);

    ROS_INFO_STREAM(name + " spawned");

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(_move_group->getPlanningFrame());
    visual_tools.trigger();

    

}