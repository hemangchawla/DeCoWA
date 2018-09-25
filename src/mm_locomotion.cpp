#include "../include/mobile_manip/mm_locomotion.h"

namespace mm_locomtion
{
bool computePath(geometry_msgs::Pose2D& start_base_pose2D, geometry_msgs::Pose2D& goal_base_pose2D, double tolerance)
{
  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", true);

  nav_msgs::GetPlan srv;

  srv.request.start.header.frame_id = "odom";
  srv.request.start.header.stamp = ros::Time::now();

  srv.request.start.pose.position.x = start_base_pose2D.x;
  srv.request.start.pose.position.y = start_base_pose2D.y;
  srv.request.start.pose.position.z = 0;

  tf::Quaternion qt_start = tf::createQuaternionFromYaw(start_base_pose2D.theta);
  srv.request.start.pose.orientation.x = qt_start.getX();
  srv.request.start.pose.orientation.y = qt_start.getY();
  srv.request.start.pose.orientation.z = qt_start.getZ();
  srv.request.start.pose.orientation.w = qt_start.getW();

  srv.request.goal.header.frame_id = "odom";
  srv.request.goal.header.stamp = ros::Time::now();

  srv.request.goal.pose.position.x = goal_base_pose2D.x;
  srv.request.goal.pose.position.y = goal_base_pose2D.y;
  srv.request.goal.pose.position.z = 0;

  tf::Quaternion qt_goal = tf::createQuaternionFromYaw(goal_base_pose2D.theta);
  srv.request.goal.pose.orientation.x = qt_goal.getX();
  srv.request.goal.pose.orientation.y = qt_goal.getY();
  srv.request.goal.pose.orientation.z = qt_goal.getZ();
  srv.request.goal.pose.orientation.w = qt_goal.getW();

  srv.request.tolerance = static_cast<float>(tolerance);  // in x and y

  if (client.call(srv))
  {
    if (!srv.response.plan.poses.empty())
    {
      return true;
    }
  }

  return false;
}

bool locomote(geometry_msgs::Pose2D& goal_base_pose2D)
{
  // TODO: use dynamic reconfiguration to set tighter tolerance

  MoveBaseClient mbc("move_base", true);

    while (ros::ok() && !mbc.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action serve to come up");
    }

    move_base_msgs::MoveBaseGoal commutation_goal;
    commutation_goal.target_pose.header.frame_id = "odom";
    commutation_goal.target_pose.header.stamp = ros::Time::now();
    commutation_goal.target_pose.pose.position.x = goal_base_pose2D.x;
    commutation_goal.target_pose.pose.position.y = goal_base_pose2D.y;
    commutation_goal.target_pose.pose.position.z = 0;
    tf::Quaternion qt = tf::createQuaternionFromYaw(goal_base_pose2D.theta);
    commutation_goal.target_pose.pose.orientation.x = qt.getX();
    commutation_goal.target_pose.pose.orientation.y = qt.getY();
    commutation_goal.target_pose.pose.orientation.z = qt.getZ();
    commutation_goal.target_pose.pose.orientation.w = qt.getW();

    mbc.sendGoal(commutation_goal);
   mbc.waitForResult();

    if (mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       ROS_INFO("Robot near dock");
       return true;
     }
     else
     {
       ROS_WARN("The robot is unable to reach near dock");
       return false;
     }

}

}  // namespace
