#include <jacob_control/planning_group.h>

int main(int argc,char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x, y, z;
    std::cin >> x >> y >> z;
    
    ROS_INFO_NAMED("teste","%f", argv[1]);

    PlanningGroup pgroup("arm");

    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
    pgroup.setGoalPosition(x,y, z);

    pgroup.moveTo(x,y,z);
    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
}