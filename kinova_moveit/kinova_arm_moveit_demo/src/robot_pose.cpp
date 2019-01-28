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
    double x, y, z, w;
    nh.getParam("x", x);
    nh.getParam("y", y);
    nh.getParam("z", z);
    nh.getParam("w", w);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //defines planning group
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //retrives robot_state info
    const robot_state::JointModelGroup* joint_model_group;
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //gets actual end_effector position
    geometry_msgs::PoseStamped now_frame = move_group.getCurrentPose();

    //pose Goal
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "root";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.w = w;

    move_group.setPoseTarget(goal); 

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

    move_group.move();

  now_frame = move_group.getCurrentPose();
    x = now_frame.pose.position.x;
    y = now_frame.pose.position.y;
    z = now_frame.pose.position.z;
    w = now_frame.pose.orientation.w; 

    ROS_INFO("Actual end-effector position is: \n x = %f, y = %f, z= %f, w= %f", x, y, z, w);  


    return 0;   
}