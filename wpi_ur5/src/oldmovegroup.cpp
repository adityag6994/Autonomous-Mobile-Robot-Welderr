#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_ur5");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(10.0);

  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


  // Defining collision object
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  // collision_object.id = "box1";

  /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.05;
  // primitive.dimensions[1] = 0.2;
  // primitive.dimensions[2] = 0.2;

  /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.0;
  // box_pose.position.y = 0.3;
  // box_pose.position.z = 0.3;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);








  sleep(10.0);

  std::vector<geometry_msgs::Pose> waypoints;
  robot_state::RobotState current_state(*group.getCurrentState());
  group.setStartState(current_state);
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose;

  start_pose.orientation.w = 1.0;
  start_pose.position.x = -0.4;
  start_pose.position.y = 0.3;
  start_pose.position.z = 0.3;

  waypoints.push_back(start_pose);

  const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose);
  group.setStartState(start_state);
  
  geometry_msgs::Pose target_pose = start_pose;
  target_pose.position.x += 0.8;
  target_pose3.position.z += 0.1;
  waypoints.push_back(target_pose);  // up and out
  target_pose3.position.y -= 0.1;
  waypoints.push_back(target_pose3);  // left
  target_pose3.position.z -= 0.1;
  target_pose3.position.y += 0.1;
  target_pose3.position.x -= 0.1;
  waypoints.push_back(target_pose3);  // down and right (back to start)
    // moveit_msgs::RobotTrajectory trajectory;
    // double fraction = group.computeCartesianPath(waypoints,
    //                                              0.01,  // eef_step
    //                                              0.0,   // jump_threshold
    //                                              trajectory);
    // ROS_INFO("Visualizing plan 1 (cartesian path) (%.2f%% acheived)",
    //       fraction * 100.0);    
    // /* Sleep to give Rviz time to visualize the plan. */

    // sleep(20.0);

 moveit::planning_interface::MoveItErrorCode errorCode = group.move();







    /*robot_state::RobotState start_state2(*group.getCurrentState());
      geometry_msgs::Pose start_pose2;
      start_pose2.orientation.w = 1.0;
      start_pose2.position.x = -0.4;
      start_pose2.position.y = 0.3;
      start_pose2.position.z = 0.3;
      const robot_state::JointModelGroup *joint_model_group2 =
                      start_state2.getJointModelGroup(group.getName());
      start_state2.setFromIK(joint_model_group2, start_pose2);
      group.setStartState(start_state2);

      ROS_INFO("Add an object into the world");
        planning_scene_interface.addCollisionObjects(collision_objects);
        sleep(2.0);

        group.setPlanningTime(50.0);

      std::vector<geometry_msgs::Pose> waypoints2;

      geometry_msgs::Pose target_pose2 = start_pose2;
      target_pose2.position.x += 0.8;
      waypoints2.push_back(target_pose2);

      moveit_msgs::RobotTrajectory trajectory2;
      double fraction2 = group.computeCartesianPath(waypoints2,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory2);

      ROS_INFO("Visualizing plan 2 (cartesian path) (%.2f%% acheived)",
            fraction2 * 100.0);
       Sleep to give Rviz time to visualize the plan.
      sleep(20.0);*/
      ros::waitForShutdown();
  //  ros::shutdown();
  return 0;
}
