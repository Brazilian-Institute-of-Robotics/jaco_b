#include <planning_group.h>

PlanningGroup::PlanningGroup(std::string name) : _group_name(name)
                                                //_robot_state(new robot_state::RobotState(_robot_model))
                                                //_move_group(new moveit::planning_interface::MoveGroupInterface(_group_name))
                                                { 
    setRobotModel();
    setPlanningScene();

    //_move_group->setGoalPositionTolerance(0.001);
    //_move_group->setGoalOrientationTolerance(0.001);
            ROS_INFO("existing");

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

void PlanningGroup::printEFPose(){
    double x = _ef_position.pose.position.x;
    double y = _ef_position.pose.position.y;
    double z = _ef_position.pose.position.z;
    double w = _ef_position.pose.orientation.w; 
    
    ROS_INFO("Actual end-effector position is: \n x = %f, y = %f, z= %f, w= %f", x, y, z, w);
}

void PlanningGroup::setGoal(geometry_msgs::Pose goal){
    _move_group->setPoseTarget(goal); 
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
