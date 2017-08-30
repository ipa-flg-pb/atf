#ifndef ROBOT_ROUTE_RECORDER_H
#define ROBOT_ROUTE_RECORDER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <fstream>

/*
 * Instructions :
 * The robot_route_recorder (rrr) is used to automatically generate robot_envs yaml files (reyf) used for the atf tests in ipa_navigation_planning_atf
 *
 * To use the rrr bring up your usual simulation (roslaunch your cob_bringup_sim robot.launch, ipa_navigation_bringup ipa_navigation.launch and ipa_navigation_bringup rviz.launch)
 * In addition start the rrr node with the robot_route_recorder.launch included in this package
 *
 * Now you only have to: - Send a 2D Pose Estimate in RVIZ; this will be the starting pose written to the reyf
 *                       - Send a number of 2D Nav Goals in RVIZ; those will define the route followed during the atf test
 *                       - Send another 2D Pose Estimate in RVIZ; this will trigger the rrr to generate the reyf from the current data and also safe the new 2D Pose Eestimate as
 *                         the starting pose for the next reyf
 *
 * Note: To modify the directory where the reyfs are saved simply modify the robot_route_recorder.launch
 */

class RobotRouteRecorder
{
private:
  // Member variables
  std::string file_directory_;
  std::string environment_name_;
  geometry_msgs::PoseWithCovariance start_pose_;
  std::vector<geometry_msgs::Pose> goals_;
  ros::NodeHandle nh_;
  ros::Subscriber start_pose_sub_, goal_sub_;
  bool has_start_pose_;
  int file_counter_;

  // Member funcions
  void startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_pose);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
public:
  RobotRouteRecorder();
  ~RobotRouteRecorder();
  void makeFileFromData();
  bool nodeOK();
};

#endif // ROBOT_ROUTE_RECORDER_H
