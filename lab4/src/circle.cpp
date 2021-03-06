#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Main moveit libraries are included
int main(int argc, char **argv)
{
ros::init(argc, argv, "move_group_interface_rectangle");
ros::NodeHandle node_handle;
ros::AsyncSpinner spinner(0);
spinner.start(); // For moveit implementation we need AsyncSpinner, we cant use ros::spinOnce()
static const std::string PLANNING_GROUP = "arm"; /* Now we
specify with what group we want work,
here group1 is the name of my group controller*/

moveit::planning_interface::MoveGroupInterface
move_group(PLANNING_GROUP); // loading move_group
const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //For joint control
geometry_msgs::PoseStamped current_pose;
current_pose = move_group.getCurrentPose();
geometry_msgs::PoseStamped target_pose= current_pose; // Pose in ROS is implemented using geometry_msgs::PoseStamped, google what is the type of this msg
/* Retrieving the
information about the
current position and orientation of the end effector*/
double x=0.0;
int y = -1;
geometry_msgs::PoseStamped origin_pose = current_pose;
//target_pose.pose.position.x = origin_pose.pose.position.x -0.01; 
//y=sqrt(1^2-(x-0.5^2))
/* Basically
our target pose is the same as current,
except that we want to move it a little bit along x-axis*/

ros::Rate loop_rate(50); //Frequency
while (ros::ok()){

    move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
    move_group.move(); // Move the robot
    current_pose = move_group.getCurrentPose();
    if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01)
    { if (abs(current_pose.pose.position.y - target_pose.pose.position.y) < 0.01) {
        if (x==0.50) {
            y=-y;  
        }
        else if (x==-0.50) {
            y=-y;                 
        }
        x=x+y*0.1;
        target_pose.pose.position.x = origin_pose.pose.position.x + x; 
        target_pose.pose.position.y = origin_pose.pose.position.y + y*sqrt(pow(0.5,2)-pow(x,2));
        ROS_INFO("%f %f", x, y*sqrt(pow(0.5,2)-pow(x,2)) );
    } }
    loop_rate.sleep();    
}
ROS_INFO("Done");
ros::shutdown();
return 0;
}