#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>
#include <ros/time.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "home_position");
    ros::NodeHandle nh;

    std::string kinova_robotName;
    trajectory_msgs::JointTrajectory msg;

    msg.header.frame_id= "1";
    msg.joint_names.resize(6);
    msg.points.resize(1);
    msg.points[0].positions.resize(6);

    double jointcmds[] = {0.0,2.9,1.3,4.2,1.4,0.0};

    //nh.getParam("kinova_robotName", kinova_robotName);

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>( "/j2n6s300/effort_joint_trajectory_controller/command", 1);

    ros::Rate loop_rate(100);

     msg.joint_names[0] = "j2n6s300_joint_1";
     msg.joint_names[1] = "j2n6s300_joint_2";
     msg.joint_names[2] = "j2n6s300_joint_3";
     msg.joint_names[3] = "j2n6s300_joint_4";
     msg.joint_names[4] = "j2n6s300_joint_5";
     msg.joint_names[5] = "j2n6s300_joint_6";
    
    msg.header.stamp =ros::Time::now() + ros::Duration(0.0);

    while(ros::ok()){

        for(int i=0; i<6; i++){
           // msg.joint_names[i] = kinova_robotName + "_joint_" + char(i+1);
            msg.points[0].positions[i] = jointcmds[i];
        }

        msg.points[0].time_from_start = ros::Duration(1.0);

        pub.publish(msg);

        loop_rate.sleep();

        ros::spinOnce();

        //ros::shutdown();
    }

    return 0;
}