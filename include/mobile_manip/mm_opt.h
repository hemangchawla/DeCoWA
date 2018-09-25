#ifndef MM_OPT_H
#define MM_OPT_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

// MOVEIT
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// EIGEN
#include <Eigen/Eigen>

// OPTIMIZATION
#include <nlopt.hpp>
#include <nlopt.h>

namespace mm_opt
{
/**
 * @brief solve Solve the optimization problem to find optimum mobile base yaw at commutation configuration
 * @param test_arm_location The test arm location is given
 * @param scene The planning scene used by moveit
 * @param link0_to_base_link The transform from link0 of the arm to base link of the mobile platform
 * @param group_name Name of the mobile base group
 * @param current_base_pose Current pose of the base
 * @return The optimum yaw angle
 */
double solve(tf::Vector3 test_arm_location, planning_scene::PlanningScenePtr& scene,
             Eigen::Affine3d& link0_to_base_link, std::string& group_name, geometry_msgs::Pose2D& current_base_pose);

// The optimization details
namespace details
{
/**
 * @brief init The optimization procedure is initialized with initial values
 * @param test_arm_tf The arm location in x,y,z
 * @param scene The planning scene used by Moveit
 * @param link0_to_base_link The transform from the link0 of the arm to the base link of the mobile platform
 * @param base_group_name Name of the mobile base group
 * @param current_base_pose Current pose2D of the robot platform
 */
void init(tf::Vector3 test_arm_tf, planning_scene::PlanningScenePtr& scene, Eigen::Affine3d& link0_to_base_link,
          std::string& base_group_name, geometry_msgs::Pose2D& current_base_pose);

/**
 * @brief generateTestStateConfiguration Generates a complete configuration for the mobile manipulator using the arm
 * location and the platform ori
 * @param angle The yaw angle of the mobile platform
 * @param whole_joint_config The joint configuration of the base plus arm is output
 */
void generateTestStateConfiguration(double angle, std::vector<double>& whole_joint_config);

/**
 * @brief objectiveFunction Objective function to optimize. Returns the distance between current and genrated base
 location, or INF if in
 collision.
 * @param x
 * @param grad
 * @param data
 * @return the Cost
 */
double objectiveFunction(const std::vector<double>& x, std::vector<double>& grad, void* data);

/**
 * @brief wrapAngle Wrap angle between 0 and 2*pi
 * @param angle Input angle
 * @return Output wrapped angle
 */
double wrapAngle(double angle);

/**
 * @brief iter Iteration of the optimization procedure
 */
unsigned int iter;
/**
 * @brief scene_ptr_ pointer to the planing scene
 */
planning_scene::PlanningScenePtr scene_ptr_;
/**
 * @brief link0_to_base_tf transform to link0 of the arm to base_link of the mobile platform
 */
Eigen::Affine3d link0_to_base_tf;
/**
 * @brief base_joint_model_group The moveit group representing the mobile base
 */
const robot_state::JointModelGroup* base_joint_model_group;
/**
 * @brief current_pose Current pose of the mobile base
 */
geometry_msgs::Pose2D current_pose;
/**
 * @brief arm_location The arm location in x,y,z
 */
tf::Vector3 arm_location;
/**
 * @brief collision_request Request for collision check of base with the scene
 */
collision_detection::CollisionRequest collision_request;
/**
 * @brief collision_result Response for collision check of the base with scene
 */
collision_detection::CollisionResult collision_result;
}
}

#endif  // MM_OPT_H
