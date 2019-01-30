#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;

    std::vector<double> joint_positions;
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;

    //get goal as parameters from launch file
    double X, Y, Z, W;
    nh.getParam("x", X);
    nh.getParam("y", Y);
    nh.getParam("z", Z);
    nh.getParam("w", W);

    geometry_msgs::PoseStamped now_frame;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //define planning group
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //retrive robot_model by its pointer 'RobotModelPtr'
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    //create planning_scene based on robot_model
    planning_scene::PlanningScene planning_scene(robot_model);

    //check if current position is in selfColision
    planning_scene.checkSelfCollision(collision_req, collision_res);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_res.collision ? "in" : "not in") << " self collision");

    //get current robot_state
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst(); 

    //print actual end-effector position
    const Eigen::Affine3d& end_effector_state = current_state.getGlobalLinkTransform("j2n6s300_end_effector");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");

    //IK
    unsigned int attempts = 11;
    double timeout = 0.3;
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    geometry_msgs::Pose pose;
    pose.position.x = X;
    pose.position.y = Y;
    pose.position.z = Z;
    pose.orientation.w = W;

    bool found_ik = current_state.setFromIK(joint_model_group, pose, "j2n6s300_end_effector", attempts, timeout);
    if(found_ik){
        current_state.copyJointGroupPositions(joint_model_group, joint_positions);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_positions[i]);
        }
    }

    //collision_detection
    planning_scene.checkSelfCollision(collision_req, collision_res, current_state);
    ROS_INFO_STREAM( (collision_res.collision ? "is in self collision" : "") );
    if(!collision_res.collision){

        //get actual end_effector position
        now_frame = move_group.getCurrentPose();
    
        //set goal position coordinates
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "root";
        goal.pose.position.x = X;
        goal.pose.position.y = Y;
        goal.pose.position.z = Z;
        goal.pose.orientation.w = W;

        move_group.setPoseTarget(goal); 

        //specify tolerances [x, y, z] -> [0.01, 0.01, 0.01]
        move_group.setGoalPositionTolerance(0.001);
        move_group.setGoalOrientationTolerance(0.001);

        //compute the plan path
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

        if(success == true){
            sleep(5.0);

            //execute the trajectory
            move_group.move();
        }
    }


    //get new position coordinates
    now_frame = move_group.getCurrentPose();
    double x = now_frame.pose.position.x;
    double y = now_frame.pose.position.y;
    double z = now_frame.pose.position.z;
    double w = now_frame.pose.orientation.w; 

    ROS_INFO("Actual end-effector position is: \n x = %f, y = %f, z= %f, w= %f", x, y, z, w);  


    return 0;   
}