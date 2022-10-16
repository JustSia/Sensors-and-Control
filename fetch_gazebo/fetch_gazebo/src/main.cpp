#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/init.h>
#include <ros/service_server.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>

#include "iksolver/calcTraj.h"

bool calcTraj(iksolver::calcTraj::Request &req,
          iksolver::calcTraj::Response &res){

  ROS_INFO("Calculate Trajectory Request Received.");
  ROS_INFO("Requested to move to:");
  ROS_INFO("x:  %f", req.coords.x);
  ROS_INFO("y:  %f", req.coords.y);
  ROS_INFO("z:  %f", req.coords.z);

  moveit::planning_interface::MoveGroupInterface move_group_interface("arm");
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup("arm");

    /* ********
  // Account for base movements
  // const Eigen::Affine3d &sensor_state = move_group_interface.getCurrentState()->getGlobalLinkTransform("head_tilt_link");
  
  // Print end-effector pose
  // ROS_INFO_STREAM("Translation: " << sensor_state.translation());
  // ROS_INFO_STREAM("Rotation: " << sensor_state.rotation());
  ******** */

  // Extract request from sensor
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = req.coords.x;
  target_pose1.position.y = req.coords.y;
  target_pose1.position.z = req.coords.z;
  move_group_interface.setPoseTarget(target_pose1);
  
  // calculate trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if( not (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) ){
    return false;
  }

  // Update robot model to reflect new state
  // move_group_interface.execute(my_plan);

  // Return calculated trajectory
  res.traj = my_plan.trajectory_;

  return true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iksolver_ex");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::ServiceServer service = node_handle.advertiseService("calc_traj", calcTraj);
  ROS_INFO("iksolver service has started");
  
  ros::waitForShutdown();
  return 0;
}