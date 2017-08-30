#include <ipa_navigation_planning_atf/robot_route_recorder.h>

RobotRouteRecorder::RobotRouteRecorder()
{
  nh_ = ros::NodeHandle("");
  start_pose_sub_ = nh_.subscribe("start_pose_topic", 1, &RobotRouteRecorder::startPoseCallback, this);
  goal_sub_ = nh_.subscribe("goals_topic", 1, &RobotRouteRecorder::goalCallback, this);
  std::vector<geometry_msgs::PoseStamped> goals_;
  has_start_pose_ = false;
  file_counter_ = 0;
  nh_.param<std::string>("/robot_route_recorder/robot_env", environment_name_, std::string("robot_env"));
  nh_.param<std::string>("/robot_route_recorder/file_directory", file_directory_, "./");
}

RobotRouteRecorder::~RobotRouteRecorder(){
}

void RobotRouteRecorder::startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_pose)
{
  if (!has_start_pose_)
  {
    start_pose_ = (*start_pose).pose;
    has_start_pose_ = true;
    ROS_INFO("RobotRouteRecorder : Received start_pose");
  }
  else
  {
    ROS_INFO("RobotRouteRecorder : Generating file from current data; then will override start_pose and clear goals");
    makeFileFromData();
    start_pose_ = (*start_pose).pose;
    goals_.clear();
  }
}

void RobotRouteRecorder::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  goals_.push_back((*goal).pose);
}

void RobotRouteRecorder::makeFileFromData()
{
  if (!has_start_pose_ || goals_.empty())
  {
    ROS_WARN("RobotRouteRecorder has insufficient data for generating an environment yaml");
    return;
  }
  std::ofstream file;
  std::string file_name = file_directory_ + environment_name_ + "-" + std::to_string(file_counter_) + ".yaml";
  file.open (file_name);

  file << "additional_parameters:\n";
  file << "    \"/atf_test/start_pose\":\n";
  file << "        \"position_x\": " + std::to_string(start_pose_.pose.position.x) +"\n";
  file << "        \"position_y\": " + std::to_string(start_pose_.pose.position.y) +"\n";
  file << "        \"orientation_x\": " + std::to_string(start_pose_.pose.orientation.x) +"\n";
  file << "        \"orientation_y\": " + std::to_string(start_pose_.pose.orientation.y) +"\n";
  file << "        \"orientation_z\": " + std::to_string(start_pose_.pose.orientation.z) +"\n";
  file << "        \"orientation_w\": " + std::to_string(start_pose_.pose.orientation.w) +"\n";
  file << "    \"/atf_test/goals\":\n";
  for (int i = 0; i < goals_.size(); i++)
  {
    file << "        - \"position_x\": " + std::to_string(goals_[i].position.x) +"\n";
    file << "          \"position_y\": " + std::to_string(goals_[i].position.y) +"\n";
    file << "          \"orientation_x\": " + std::to_string(goals_[i].orientation.x) +"\n";
    file << "          \"orientation_y\": " + std::to_string(goals_[i].orientation.y) +"\n";
    file << "          \"orientation_z\": " + std::to_string(goals_[i].orientation.z) +"\n";
    file << "          \"orientation_w\": " + std::to_string(goals_[i].orientation.w) +"\n";
  }
  file << "additional_arguments:\n";
  file << "    \"robot_env\": " + environment_name_ + "\n";

  file.close();
  file_counter_++;
}

bool RobotRouteRecorder::nodeOK()
{
  return nh_.ok();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_route_recorder");
  RobotRouteRecorder rrr;
  ros::Rate loop_rate(10.0);
  while (rrr.nodeOK())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Dropping current data without generating an environment yaml");
  return 0;
}
