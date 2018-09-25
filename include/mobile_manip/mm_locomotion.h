#ifndef MM_LOCOMOTION_H
#define MM_LOCOMOTION_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

// Navigation stack
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>


namespace mm_locomtion
{
/**
 * @brief computePath Compute the path from start pose to goal pose within the xy tolerance
 * @param start_base_pose2D The Pose2D for starting
 * @param goal_base_pose2D The Pose2D for goal
 * @param tolerance The tolerance in x and y coordinates around the goal for finding the path
 * @return True if a path is found, else false
 */
bool computePath(geometry_msgs::Pose2D& start_base_pose2D, geometry_msgs::Pose2D& goal_base_pose2D, double tolerance);

/**
 * @brief locomote The behavior to make the robot navigate to computed robot placement
 * @param goal_base_pose2D The pose2D goal based on computed robot placement
 * @return true if the robot successfully reaches the goal
 */
bool locomote(geometry_msgs::Pose2D& goal_base_pose2D);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

}
#endif // MM_LOCOMOTION_H
