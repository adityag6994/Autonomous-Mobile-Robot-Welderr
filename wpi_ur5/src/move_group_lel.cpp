#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

void percCallback(const geometry_msgs::PoseArray& points){
	

	moveit::planning_interface::MoveGroup group("manipulator");
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	sleep(10.0);

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.clear();

	robot_state::RobotState current_state(*group.getCurrentState());
	group.setStartState(current_state);
	robot_state::RobotState start_state(*group.getCurrentState());
	  
	int size = points.poses.size();
	for(int i=0;i<=size;i++){
		geometry_msgs::Pose targetPose;
		waypoints.push_back(points.poses[i]);
        }
	ROS_INFO("Received msg size : %i",waypoints.size()); 
	
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

	ROS_INFO("Visualizing plan (%.2f%% acheived)", fraction * 100.0);

	/* Sleep to give Rviz time to visualize the plan. */
	sleep(15.0);
	moveit::planning_interface::MoveItErrorCode errorCode = group.move();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "move_group_ur5");
  	ros::NodeHandle node_handle;  
  	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	
  	sleep(10.0);

	ros::Subscriber sub = node_handle.subscribe("percTopic",10, percCallback);
	ros::spin();
	ros::waitForShutdown();
	return 0;
}