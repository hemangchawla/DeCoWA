#ifndef MM_MANIPULATION_H
#define MM_MANIPULATION_H

// Basic
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>

// Eigen
#include <Eigen/Core>

// VREP
#include <vrep_common/JointSetStateData.h>

namespace mm_manipulation
{
// Rate at which the manipulation will be executed
unsigned int SPIN_RATE = 30;

/**
 * @brief unfoldArm To unfold the arm to default start position before beginning the motion planning to target pose
 * @param  folded_positions The configuration of arm when it is folded
 * @param unfolded_positions The configuration of arm when it is unfolded
 * @param joint_names The names of the joints of the kinematic chain in the moveit srdf
 * @param simulation_publish Set to true if simulation is to be published on VREP. Default is false
 * @param joint_handles These are the handles used by VREP. Default is list of zero.
 * @return true if the arm successfully unfolds
 */
bool unfoldArm(std::vector<double>& folded_positions, std::vector<double>& unfolded_positions,
               std::vector<std::string>& joint_names, bool simulation_publish = false,
               std::vector<int> joint_handles = { 0 });
/**
 * @brief computeMotionPlan Compute the motion plan of the arm
 * @param group The arm group in moveit
 * @param start_state The begining state of the arm
 * @param target The target pose for the end effector
 * @return bool is a successful motion plan could be found within the time constraints
 */
bool computeMotionPlan(moveit::planning_interface::MoveGroupInterfacePtr& group, robot_state::RobotState& start_state,
                       geometry_msgs::Pose& target);

/**
 * @brief executeMotion To make the robot arm move, position commands are sent at a constant interval
 * @param trajectory_msg The generated trajectory which the robotic arm has to trace
 * @param simulation_publish Set to true if simulation is to be published on VREP. Default is false.
 * @param publishing_rate The rate at which the position commands would be sent to the robotic arm for motion execution.
 * Default is the same SPIN_RATE as above
 * @param joint_handles These are the handles used by VREP. Default is a list of zero.
 * @return true if the motion execution of the arm led it successfully to the target pose
 */
bool executeMotion(moveit_msgs::DisplayTrajectory& trajectory_msg, bool simulation_publish = false,
                   unsigned int publishing_rate = SPIN_RATE, std::vector<int> joint_handles = { 0 });
}

#endif  // MM_MANIPULATION_H
