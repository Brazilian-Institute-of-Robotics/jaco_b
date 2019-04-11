#include <jacob_control/PlanningGroup.h>
#include <jacob_control/ObjectCreator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc,char** argv){
    ros::init(argc, argv, "moveto");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    double x, y, z;
    std::cin >> x >> y >> z;

    PlanningGroup pgroup("arm");

    pgroup.actualizeEFPosition();
    pgroup.printEFPose();

    ObjectCreator box(BOX, "plan", nh); //the same from srdf
 

    pgroup.setGoalPosition(x,y, z);

    pgroup.moveTo(x,y,z);
    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
}