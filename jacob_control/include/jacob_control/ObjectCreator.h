#include <moveit_msgs/CollisionObject.h>
#include <jacob_control/PlanningGroup.h>



enum Geometry{
        BOX
    };

class ObjectCreator : public PlanningGroup{

    public:

    ObjectCreator(Geometry type, std::string name);
    void setPosition( moveit_msgs::CollisionObject object);
    void setPrimitive(moveit_msgs::CollisionObject object, Geometry type);
    void createObject(Geometry type, std::string name);
    static std::string _name;


    private:
    std::vector<moveit_msgs::CollisionObject> _collision_objects;
    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
    moveit_msgs::CollisionObject _object;
    shape_msgs::SolidPrimitive _type;
    geometry_msgs::Pose _pose;


};

