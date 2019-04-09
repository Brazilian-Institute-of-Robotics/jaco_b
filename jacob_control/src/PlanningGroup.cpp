#include <jacob_control/PlanningGroup.h>
#include <memory>

<<<<<<< HEAD:jacob_control/src/planning_group.cpp
PlanningGroup::PlanningGroup(std::string name) : _group_name(name),
                                                //_robot_state(new robot_state::RobotState(_robot_model))
                                                //move_group(new moveit::planning_interface::MoveGroupInterface(_group_name))
                                                _move_group (std::make_shared<moveit::planning_interface::MoveGroupInterface>(_group_name))
                                                { 
=======
PlanningGroup::PlanningGroup(std::string name){
                                                
    _group_name = name;
    _move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(_group_name);
>>>>>>> 7a78e7a... add ObjectCreator class:jacob_control/src/PlanningGroup.cpp
    setRobotModel();
    setPlanningScene();

    _move_group->setGoalPositionTolerance(0.001);
    _move_group->setGoalOrientationTolerance(0.001);

}

void PlanningGroup::actualizeEFPosition(){
    _ef_position = _move_group->getCurrentPose();
}

bool PlanningGroup::checkGoal(geometry_msgs::Pose goal){
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

    return success;
}

void PlanningGroup::checkSelfCollision(){
    _planning_scene->checkSelfCollision(_collision_req, _collision_res);
    ROS_INFO_STREAM("Test 1: Current state is " << (_collision_res.collision ? "in" : "not in") << " self collision");
}

void PlanningGroup::moveTo(geometry_msgs::Pose goal){
    setGoal(goal);
    if ( checkGoal(goal) == true ){
        sleep(5.0);

        //execute the trajectory
        _move_group->move();
    }
}

void PlanningGroup::moveTo(double x, double y, double z){
    tf::Pose t_goal;
    t_goal.setOrigin( tf::Vector3(x, y, z) );
    t_goal.setRotation( tf::Quaternion(-0.5832, 0.6325, 0.29265, 0.41628));
    tf::poseTFToMsg(t_goal, _goal);
    moveTo(_goal);
}

void PlanningGroup::printEFPose(){
    double x = _ef_position.pose.position.x;
    double y = _ef_position.pose.position.y;
    double z = _ef_position.pose.position.z;
    double w = _ef_position.pose.orientation.w; 
    
    ROS_INFO("Actual end-effector position is: \n x = %f, y = %f, z= %f, w= %f", x, y, z, w);
}

std::string PlanningGroup::getGroupName(){
    return _group_name;
}

std::string PlanningGroup::getPlanningFrame(){
    return _move_group->getPlanningFrame();
}

void PlanningGroup::setGoal(geometry_msgs::Pose goal){
    _move_group->setPoseTarget(goal); 
}

void PlanningGroup::setGoalPosition(double x, double y, double z ){
    tf::Pose t_goal;
    t_goal.setOrigin( tf::Vector3(x, y, z) );
    t_goal.setRotation( tf::Quaternion(-0.5832, 0.6325, 0.29265, 0.41628));

    tf::poseTFToMsg(t_goal, _goal);
}

void PlanningGroup::setRobotModel(){
    ROS_INFO("existing");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    _robot_model = robot_model_loader.getModel();
    ROS_INFO("existing");
}

void PlanningGroup::setRobotState(){
}

void PlanningGroup::setPlanningScene(){
   _planning_scene = new planning_scene::PlanningScene(_robot_model);
}
