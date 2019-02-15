#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_datatypes.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_pose");
    ros::NodeHandle nh;

    std::vector<double> joint_positions;
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
    const Eigen::Affine3d& end_effector_state = current_state.getGlobalLinkTransform(move_group.getEndEffectorLink());
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    
    //set goal position coordinates
    tf::Pose t_goal;
    t_goal.setOrigin( tf::Vector3(X, Y, Z) );
    t_goal.setRotation( tf::Quaternion(-0.5832, 0.6325, 0.41627, 0.41628));

    geometry_msgs::Pose goal;

    tf::poseTFToMsg(t_goal, goal);


    //specify tolerances [x, y, z] -> [0.01, 0.01, 0.01]
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.001);

    move_group.setPoseTarget(goal); 
//----------------------------------------------- add ground collision --------------------------------------------------------
    //collision object declaration
    moveit_msgs::CollisionObject plane;
    plane.id = "plane";
    plane.header.frame_id = move_group.getPlanningFrame();
 
    //primitive especification
    shape_msgs::SolidPrimitive plane_primitive;
    plane_primitive.type = plane_primitive.BOX;
    plane_primitive.dimensions.resize(3);
    plane_primitive.dimensions[0] = 3.0;
    plane_primitive.dimensions[1] = 3.0;
    plane_primitive.dimensions[2] = 0.0;


    //object position especification
    geometry_msgs::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.x = 0.0;
    plane_pose.position.y = 0.0;
    plane_pose.position.z = 0.0;

    plane.primitives.push_back(plane_primitive);
    plane.primitive_poses.push_back(plane_pose);
    plane.operation = plane.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(plane);

    //add collision object into planning_scene
    planning_scene_interface.addCollisionObjects(collision_objects);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group.getPlanningFrame());
    visual_tools.trigger();
    
//----------------------------------------------------------------- Plan Path -------------------------------------------------------------
    //compute the plan path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

    if(success == true){
            sleep(5.0);

            //execute the trajectory
            move_group.move();
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