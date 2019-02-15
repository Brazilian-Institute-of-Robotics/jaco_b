#define _USE_MATH_DEFINES
#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "teste_joint");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_interface::MotionPlanRequest motion_req;
    planning_interface::MotionPlanResponse motion_res;

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group.getPlanningFrame());
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[1] = 0.4; // lower than joint_2_lower_limit
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();
/*//------------------------------------------------------------ planning scene -------------------------------------------------------------------------

planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

//------------------------------------------------------------ planning plugin --------------------------------------------------------------

//boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
//std::string planner_plugin_name;

//------------------------------------------------------------ Motion Plan Request -------------------------------------------------------------

robot_state::RobotState goal_state(robot_model);
std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

motion_req.group_name = PLANNING_GROUP;
motion_req.goal_constraints.clear();
motion_req.goal_constraints.push_back(joint_goal);

//------------------------------------------------------------- Planning Context ------------------------------------------------------------
planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, motion_req, motion_res.error_code_);

context->solve(motion_res);

if (motion_res.error_code_.val != motion_res.error_code_.SUCCESS)
{
    ROS_ERROR("Could not compute plan successfully");
    return 0;
}
*/
return 0;
}
