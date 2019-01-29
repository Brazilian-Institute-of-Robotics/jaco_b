#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;

    //get parameter from launch file
    double X = 0, Y = 0, Z = 0, W = 0;
    nh.getParam("x", X);
    nh.getParam("y", Y);
    nh.getParam("z", Z);
    nh.getParam("w", W);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //define planning group
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //retrive robot_state info
    const robot_state::JointModelGroup* joint_model_group;
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //get actual end_effector position
    geometry_msgs::PoseStamped now_frame = move_group.getCurrentPose();
    
    //set goal position coordinates
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "root";
    goal.pose.position.x = X;
    goal.pose.position.y = Y;
    goal.pose.position.z = Z;
    goal.pose.orientation.w = W;

    move_group.setPoseTarget(goal); 

    //specify tolerances [x, y, z] -> [0.01, 0.01, 0.01]
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.001);

    //compute the plan path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

    sleep(5.0);

    //execute the trajectory
    move_group.move();

    //get new position coordinates
    now_frame = move_group.getCurrentPose();
    double x = now_frame.pose.position.x;
    double y = now_frame.pose.position.y;
    double z = now_frame.pose.position.z;
    double w = now_frame.pose.orientation.w; 

    ROS_INFO("Actual end-effector position is: \n x = %f, y = %f, z= %f, w= %f", x, y, z, w);  


    return 0;   
}