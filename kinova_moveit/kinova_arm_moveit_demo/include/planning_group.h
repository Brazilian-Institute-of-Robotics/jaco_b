//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_datatypes.h>




class PlanningGroup {
    public:

    PlanningGroup(std::string name);
    void actualizeEFPosition();
    void checkSelfCollision();
    bool checkGoal(geometry_msgs::Pose goal);
    void moveTo(geometry_msgs::Pose goal);
    void printEFPose();
    void setRobotModel();
    void setRobotState();
    void setPlanningScene();
    void setGoal(geometry_msgs::Pose goal);
    void setGoalPosition(double x, double y, double z);
    void setGoalPosition(tf::Vector3 pose);
    void setGoalOrientation(double x, double y, double z, double w);
    void setGoalOrientation(double r, double p, double y);
    void setGoalOrientation(tf::Quaternion q);

    private:

    std::string _group_name;
    std::vector<double> _joint_positions;
    geometry_msgs::PoseStamped _ef_position;
    geometry_msgs::Pose _goal;
    moveit::planning_interface::MoveGroupInterface* _move_group; //todo change all raw pointers to shared_ptr
    robot_model::RobotModelPtr _robot_model;
    robot_state::RobotStatePtr _robot_state;
    planning_scene::PlanningScene* _planning_scene;
    collision_detection::CollisionRequest _collision_req;
    collision_detection::CollisionResult _collision_res; 
};