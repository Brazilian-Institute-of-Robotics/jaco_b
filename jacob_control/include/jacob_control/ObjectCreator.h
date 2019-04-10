#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <jacob_control/PlanningGroup.h>
#include <ros/ros.h>



enum Geometry{
        BOX
    };

class ObjectCreator : public PlanningGroup{

    public:

    ObjectCreator(Geometry type, std::string name, ros::NodeHandle node);
    void setPosition( moveit_msgs::CollisionObject& object);
    void setPrimitive(moveit_msgs::CollisionObject& object, Geometry type);
    void createObject(Geometry type, std::string name, ros::NodeHandle node);
    static std::string _name;
    moveit_msgs::PlanningScene _planning_scene;
    ros::Publisher planning_scene_diff_publisher;


    private:
    moveit_msgs::CollisionObject _collision_object;
    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
    moveit_msgs::CollisionObject _object;
    shape_msgs::SolidPrimitive _type;
    geometry_msgs::Pose _pose;


};

