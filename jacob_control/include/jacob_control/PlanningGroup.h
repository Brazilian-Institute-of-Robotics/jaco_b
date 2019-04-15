//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#ifndef PLANNING_GROUP_H 
#define PLANNING_GROUP_H


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/transform_datatypes.h>


class PlanningGroup {
    public:

    PlanningGroup(std::string name);
    void actualizeEFPosition();
    void checkSelfCollision();
    bool checkGoal();
    void moveTo(geometry_msgs::PoseStamped goal);
    void moveTo(geometry_msgs::Pose goal);
    void moveTo(double x, double y, double z);
    void printEFPose();
    void printGoalPosition();
    std::string getGroupName();
    std::string getPlanningFrame();
    void setRobotModel();
    void setRobotState();
    void setPlanningScene();
    void setGoal(geometry_msgs::Pose goal);
    void setGoal(geometry_msgs::PoseStamped goal);
    void setGoalPosition(double x, double y, double z);
    void setGoalPosition(tf::Vector3 pose);
    void setGoalOrientation(double x, double y, double z, double w);
    void setGoalOrientation(double r, double p, double y);
    void setGoalOrientation(tf::Quaternion q);

    geometry_msgs::Pose _goal;

    protected:

    std::string _group_name;
    std::vector<double> _joint_positions;
    geometry_msgs::PoseStamped _ef_position;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group;
    robot_model::RobotModelPtr _robot_model;
    robot_state::RobotStatePtr _robot_state;
    planning_scene::PlanningScene* _planning_scene;
    collision_detection::CollisionRequest _collision_req;
    collision_detection::CollisionResult _collision_res; 
};

#endif