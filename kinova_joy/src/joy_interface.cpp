#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

double analogic_x = 0.0;
double analogic_y = 0.0;
double channel_;

int actualize;

float pose[6] = {0.0, 2.9, 1.3, 4.2, 1.4, 0.0};
float joints[6] = {0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};

void joyReceiver(const sensor_msgs::Joy& msg){
    analogic_x = msg.axes[0]*0.2; 
    analogic_y = msg.axes[1]*0.2;

    actualize = msg.buttons[0];

    channel_ = msg.axes[6]*(-1);

}
/*
void jointState(const sensor_msgs::JointState& state){
    for(int i=0; i<6; i++){
        joints[i] = state.position[i];
    }
}
*/

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle nh;
    ROS_INFO("Hello");
    //It specifies if command interface is on/off
    bool status = false;

    // channel will specify wich of the 6 joints is receiving command
    int channel = 0; 

    //Here I'm getting joy output and making it a joint_controller input 
    ros::Publisher joint_1 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_1_position_controller/command", 1000);
    ros::Publisher joint_2 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_2_position_controller/command", 1000);
    ros::Publisher joint_3 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_3_position_controller/command", 1000);
    ros::Publisher joint_4 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_4_position_controller/command", 1000);
    ros::Publisher joint_5 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_5_position_controller/command", 1000);
    ros::Publisher joint_6 = nh.advertise<std_msgs::Float64>("j2n6s300/joint_6_position_controller/command", 1000);

    ros::Subscriber joy = nh.subscribe("joy", 1000, joyReceiver);
    //ros::Subscriber state = nh.subscribe("j2n6s300/joint_states", 1000, jointState); 

    //Publishing messages at /joy rate
    ros::Rate loop_rate(10);

    //The message object to receive joy output
    sensor_msgs::Joy msg;

    //The message objects to send command controller
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;

    while(ros::ok()){
        if(channel_ != 0.){
            channel = channel + (int)channel_;
            channel = (channel % 6);
            if(channel <= 0){
                channel = channel + 6;
            }
        }
        ROS_INFO("Channel %d", channel);
    
        if(actualize == 1){
            ROS_INFO("Status %d", status);
            status = !status;
        }

        if(status == true){
            switch(channel){
                case 1:
                    ROS_INFO("Joint %d", channel);

                    if( fabs(analogic_x) == 0.2){
                        msg1.data = analogic_x + pose[channel-1];
                        pose[channel-1] = msg1.data;
                    }
                    else{
                        msg1.data = pose[channel-1];
                    }
                    joint_1.publish(msg1);


                    ROS_INFO("Data %f", fabs(analogic_x));
                break;

                case 2:
                    ROS_INFO("Joint %d", channel);
                    if( fabs(analogic_y) == 0.2){
                        msg2.data = analogic_y + pose[channel-1];
                        pose[channel-1] = msg2.data;
                        ROS_INFO("data = %f", msg2.data);
                    }
                    else{
                        msg2.data = pose[channel-1];
                    }
                    joint_2.publish(msg2);
                break;

                case 3:
                    ROS_INFO("Joint %d", channel);
                    if( fabs(analogic_y) == 0.2){
                        msg2.data = analogic_y*0.2 + pose[channel-1]; //making joint_3 moves at 30% of other joints speed
                        pose[channel-1] = msg2.data;
                    }
                    else{
                        msg2.data = pose[channel-1];
                    }
                    joint_3.publish(msg2);
                break;

                case 4:
                    ROS_INFO("Joint %d", channel);
                    if( fabs(analogic_y) == 0.2){
                        msg2.data = analogic_y + pose[channel-1];
                        pose[channel-1] = msg2.data;
                    }
                    else{
                        msg2.data = pose[channel-1];
                    }
                    joint_4.publish(msg2);
                break;

                case 5:
                    ROS_INFO("Joint %d", channel);
                    if( fabs(analogic_y) == 0.2){
                        msg2.data = analogic_y + pose[channel-1];
                        pose[channel-1] = msg2.data;
                    }
                    else{
                        msg2.data = pose[channel-1];
                    }
                    joint_5.publish(msg2);
                break;

                case 6:
                    ROS_INFO("Joint %d", channel);
                    if( fabs(analogic_y) == 0.2){
                        msg2.data = analogic_y + pose[channel-1];
                        pose[channel-1] = msg2.data;
                    }
                    else{
                        msg2.data = pose[channel-1];
                    }
                    joint_6.publish(msg2);
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}