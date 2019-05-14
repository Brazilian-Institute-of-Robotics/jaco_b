#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajectoryClient;


class RobotArm
{
    private:
    TrajectoryClient* _traj_client;
    ros::Subscriber _joint_sub;
    ros::NodeHandle _nh;
    float _joint_position[7];

    public:
    RobotArm()
    {
        ros::NodeHandle nh;
        _traj_client = new TrajectoryClient("jacob/effort_joint_trajectory_controller/follow_joint_trajectory", true);
        _joint_sub = _nh.subscribe("/joint_state", 10, &RobotArm::joint_callback, this);

        while(!_traj_client->waitForServer(ros::Duration(5.0)) && ros::ok()){
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }

    ~RobotArm()
    {
        delete _traj_client;
    }

    void joint_callback(const std_msgs::Float64MultiArray &msg)
    {
        for (int i = 0; i<7; i++)
        {
            _joint_position[i] = msg.data[i];
            ROS_INFO("joint = %f", _joint_position[i]);

        }
    }

    //Sends the command to start a given trajectory
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        //Start the trajectory NOW
        goal.trajectory.header.stamp = ros::Time::now();
        _traj_client->sendGoal(goal);
    }

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
    {
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("jacob_joint_1");
        goal.trajectory.joint_names.push_back("jacob_joint_2");
        goal.trajectory.joint_names.push_back("jacob_joint_3");
        goal.trajectory.joint_names.push_back("jacob_joint_4");
        goal.trajectory.joint_names.push_back("jacob_joint_5");
        goal.trajectory.joint_names.push_back("jacob_joint_6");
        //goal.trajectory.joint_names.push_back("jacob_joint_finger_1");

        goal.trajectory.points.resize(1);

        goal.trajectory.points[0].positions.resize(6);
        goal.trajectory.points[0].positions[0] = _joint_position[0];
        goal.trajectory.points[0].positions[1] = _joint_position[1];
        goal.trajectory.points[0].positions[2] = _joint_position[2];
        goal.trajectory.points[0].positions[3] = _joint_position[3];
        goal.trajectory.points[0].positions[4] = _joint_position[4];
        goal.trajectory.points[0].positions[5] = _joint_position[5];
        goal.trajectory.points[0].positions[6] = _joint_position[6];        

        //velocities
        goal.trajectory.points[0].velocities.resize(6);

        for (size_t j=0; j <6; ++j)
        {
            goal.trajectory.points[0].velocities[j] =0.0;
        }
        //to be reached 2 second after starting
        goal.trajectory.points[0].time_from_start = ros::Duration(0.02);

        return goal;
    }

    //Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        return _traj_client->getState();
    }
};

int main (int argc, char** argv){
    ros::init(argc, argv, "robot_driver");
    RobotArm jacob;
    //jacob.startTrajectory(jacob.armExtensionTrajectory());

    while(ros::ok())
    {   
        ros::spinOnce();
        jacob.startTrajectory(jacob.armExtensionTrajectory());
        //ros::Duration(200000).sleep();
    }
    return 0;
}