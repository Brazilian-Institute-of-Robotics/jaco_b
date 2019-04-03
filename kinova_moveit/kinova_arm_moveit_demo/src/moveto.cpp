#include <planning_group.h>

int main(int argc,char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf::Pose t_goal;
    t_goal.setOrigin( tf::Vector3(-2.3, 0.6, 0.8) );
    t_goal.setRotation( tf::Quaternion(-0.5832, 0.6325, 0.41627, 0.41628));

    geometry_msgs::Pose goal;
    tf::poseTFToMsg(t_goal, goal);
    PlanningGroup pgroup("arm");
    pgroup.printEFPose();
            ROS_INFO("here1");

    //pgroup.moveTo(goal);
    //pgroup.actualizeEFPosition();
    pgroup.setGoal(goal);
            ROS_INFO("here2");

    pgroup.printEFPose();

    //ROS_INFO("existing");

}