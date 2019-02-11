#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"


float pose[6] = {0.0, 2.9, 1.3, 4.2, 1.4, 0.0};
float f_pose[3] = {1.0, 1.0, 1.0};
//float pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};
float states[6];

void jointState(const sensor_msgs::JointState& state){
    for(int i=0; i<6; i++){
        states[i] = state.position[i];
    }
}

void shutdowAlarm(const ros::TimerEvent&){
    ROS_INFO("FINISHING");
    ros::shutdown();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle nh;

    //Here I'm getting joy output and making it a joint_controller input 
    ros::Publisher joint_1 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_1_position_controller/command", 1000);
    ros::Publisher joint_2 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_2_position_controller/command", 1000);
    ros::Publisher joint_3 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_3_position_controller/command", 1000);
    ros::Publisher joint_4 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_4_position_controller/command", 1000);
    ros::Publisher joint_5 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_5_position_controller/command", 1000);
    ros::Publisher joint_6 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_6_position_controller/command", 1000);

    ros::Publisher finger_1 = nh.advertise<std_msgs::Float64>("j2n6s300/finger_1_position_controller/command", 1000);
    ros::Publisher finger_2 = nh.advertise<std_msgs::Float64>("j2n6s300/finger_2_position_controller/command", 1000);
    ros::Publisher finger_3 = nh.advertise<std_msgs::Float64>("j2n6s300/finger_3_position_controller/command", 1000);

    ros::Subscriber state = nh.subscribe("j2n6s300/joint_states", 1000, jointState);

    ros::Timer timer = nh.createTimer(ros::Duration(2.0), shutdowAlarm);

    //Publishing messages at ... | default is /joy rate = loop_rate(10);
    ros::Rate loop_rate(0.2);

    //The message objects to send command controller
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;
    std_msgs::Float64 msg4;
    std_msgs::Float64 msg5;
    std_msgs::Float64 msg6;

    std_msgs::Float64 fmsg1;
    std_msgs::Float64 fmsg2;
    std_msgs::Float64 fmsg3;

    while(ros::ok()){
        ROS_INFO("rUNNING");
        bool finish;
        msg1.data = pose[0];
        msg2.data = pose[1];
        msg3.data = pose[2];
        msg4.data = pose[3];
        msg5.data = pose[4];
        msg6.data = pose[5];

        fmsg1.data = f_pose[0];
        fmsg2.data = f_pose[1];
        fmsg3.data = f_pose[2];

        joint_1.publish(msg1);
        joint_2.publish(msg2);
        joint_3.publish(msg3);
        joint_4.publish(msg4);
        joint_5.publish(msg5);
        joint_6.publish(msg6);
        finger_1.publish(fmsg1);
        finger_2.publish(fmsg2);
        finger_3.publish(fmsg3);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}