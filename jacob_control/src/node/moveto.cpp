#include <jacob_control/planning_group.h>

int main(int argc,char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x, y, z;
    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);

    PlanningGroup pgroup("arm");

    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
    pgroup.setGoalPosition(x,y, z);

    pgroup.moveTo(x,y,z);
    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
}