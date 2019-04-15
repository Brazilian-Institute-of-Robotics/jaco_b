#include <jacob_control/PlanningGroup.h>
#include <jacob_control/ObjectCreator.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::TransformStamped getMarkTf(){
    bool tag_exists = false;
    geometry_msgs::TransformStamped tfTag;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while(!tag_exists){
        if (tfBuffer._frameExists("tag_0")){
            tag_exists = true;
            try{
                tfTag = tfBuffer.lookupTransform("j2s6s300_end_effector", "tag_0", ros::Time(0));
            }
            catch (tf2::TransformException &ex){
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
    }
    return tfTag;
}

geometry_msgs::PoseStamped getMarkPose(geometry_msgs::TransformStamped tfTag){
    geometry_msgs::PoseStamped closeUp, closeUpWorld;
    closeUp.pose.position.x = 0.0;
    closeUp.pose.position.y = 0.0;
    closeUp.pose.position.z = 0.4;
    geometry_msgs::Quaternion q_msg;
    tf::Quaternion qt = tf::createQuaternionFromRPY(M_PI, 0, 0);
    closeUp.pose.orientation = q_msg;
    closeUp.header.stamp = ros::Time::now();
    closeUp.header.frame_id = "tag_0";
    tf2::doTransform(closeUp, closeUpWorld, tfTag);
    return closeUpWorld;
}

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

    ObjectCreator box(BOX, "plan", nh);
 

    pgroup.setGoal(getMarkPose(getMarkTf()));

    pgroup.moveTo(x,y,z);
    pgroup.actualizeEFPosition();
    pgroup.printEFPose();
}