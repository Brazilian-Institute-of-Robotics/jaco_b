#include <jacob_control/PlanningGroup.h>
#include <jacob_control/ObjectCreator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc,char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    double x, y, z;
<<<<<<< HEAD
    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);
=======
    //std::cin >> x >> y >> z;
>>>>>>> 7a78e7a... add ObjectCreator class

    PlanningGroup pgroup("arm");

    //pgroup.actualizeEFPosition();
    //pgroup.printEFPose();

    ObjectCreator box(BOX, "arm"); //the same from srdf
 

    //pgroup.setGoalPosition(x,y, z);

    //pgroup.moveTo(x,y,z);
    //pgroup.actualizeEFPosition();
    //pgroup.printEFPose();
}