#ifndef MOBILE_MANIPULATION_H
#define MOBILE_MANIPULATION_H

// BASIC
#include <ctime>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// V-REP
#include <vrep_common/JointSetStateData.h>

// Libraries
#include "mm_robot_placement.h"
#include "mm_manipulation.h"
#include "mm_locomotion.h"

#define SPIN_RATE 5  // used for ros-spin

/**
 * @brief The mobile manipulation class
 */
class MobileManipulation
{
public:
  // Functions
  // Constructor
  MobileManipulation();
  // Destructor
  ~MobileManipulation();
  /**
   * @brief spin The main control function that handles the state machine
   */
  void spin();


  // Variables

  /**
   * @brief The StatesMobileManipulation enum The states in which the mobile manipultion procedure can be
   */
  enum StatesMobileManipulation
  {
    IDLE,                      // In the beginning
    COMPUTING_TARGET,          // Based on the vision system
    COMPUTING_BASE_PLACEMENT,  // Based on the 2D and 3D map and the IRM, IPM maps
    COMPUTING_BASE_MOTION,     // Using Navigation Stack
    EXECUTING_BASE_MOTION,     // Navigation Stack as well
    COMPUTING_PRE_GRASP_POSE,  // Genetic Search, or take same as target pose
    COMPUTING_ARM_MOTION,      // Using MoveIt!
    UNFOLDING_ARM,             // To pre-determined configuration. Assumption that unfolding is possible
    EXECUTING_ARM_MOTION,      // Using CAN protocol
    COMPUTING_GRASP,           // TODO
    EXECUTING_GRASP,           // TODO
    COMPLETE,
  };

  /**
   * @brief The EndingState enum The state at which the mobile manipulation cycle exited
   */
  enum EndingState
  {
    SUCCESS,
    TARGET_NOT_RECEIVED,
    COMPUTING_COMMUTATION_CONFIGURATION_FAIL,
    COMPUTING_BASE_MOTION_FAIL,
    EXECUTING_BASE_MOTION_FAIL,
    PRE_GRASP_COMPUTATION_FAIL,
    UNFOLDING_ARM_FAIL,
    COMPUTING_ARM_MOTION_FAIL,
    EXECUTING_ARM_MOTION_FAIL,
    /// @todo: Add endings for grasping
  };

private:
  // FUNCTIONS
  /**
   * @brief loadParameters Load the ROS parameters from yaml config file
   */
  void loadParameters(void);

  /**
   * @brief setEndEffectorTarget Integrates with vision system to determine the target pose
   * @param msg the Pose msg used to set target of end effector
   */
  void setEndEffectorTarget(const geometry_msgs::Pose msg);

  /**
   * @brief setExecutionMode Simulation or Real
   */
  void setExecutionMode(void);

  /**
   * @brief setMoveGroupNames Set name of the moveit move groups: arm, mobile platform, and whole body
   */
  void setMoveGroupNames(void);

  /**
   * @brief stateMachineMobileManipulation This function keeps spinning maintaining the state of the machine
   */
  void stateMachineMobileManipulation();

  /**
   * @brief setInverseMapType Set the inverse map type i.e. REACHABILITY, PLANABILITY, FUSION
   */
  void setInverseMapType(void);

  /**
   * @brief get2DMap Read the 2D cost map at the instant
   * @param msg Costmap2D msg
   */
  void get2DMap(const nav_msgs::OccupancyGrid& msg);

  /**
   * @brief getVrepJointHandles  Read the joint handle numbers from vrep for the arm joints
   * @param msg JointHandle msg in Int32MultiArray format
   */
  void getVrepJointHandles(const std_msgs::Int32MultiArray& msg);

  /**
   * @brief getCurrentPlatformPose  Read the location of the platfrom from odom/vrep
   * @param msg Odom msg
   */
  void getCurrentPlatformPose(const geometry_msgs::Pose2D& msg);

  /**
   * @brief readArmTrajectory Read and store the computed planner trajectory from moveit
   * @param msg Moveit trajectory msg
   */
  void readArmTrajectory(const moveit_msgs::DisplayTrajectory& msg);

  // VARIABLES

  // ROS subscribers, publishers and node handles
  ros::NodeHandle nh_;
  ros::Subscriber Subscriber_end_effector_target_;
  ros::Subscriber Subscriber_arm_path_;
  ros::Subscriber Subscriber_2D_map_;
  ros::Subscriber Subscriber_vrep_joint_handles_;
  ros::Subscriber Subscriber_odom_;

  // Params from yaml file
  std::string execution_mode_;
  std::string map_type_;
  std::string whole_robot_group_;
  std::string platform_group_;
  std::string arm_group_;
  std::string arm_link0_name_;
  std::string zero_state_name_;
  std::string unfolded_state_name_;

  // bool variables for keeping track of progress
  bool simulation_mode_;
  bool end_effector_target_pose_rcvd_;
  bool computing_base_motion_successful_;
  bool executing_base_motion_successful_;
  bool pre_grasp_pose_rcvd_;
  bool unfolding_arm_successful_;
  bool computing_arm_motion_successful_;
  bool executing_arm_motion_successful_;

  /**
   * @brief state_mobile_manipulation_ State of the mobile manipulation is tracked as the mobile manipulation happens
   */
  StatesMobileManipulation state_mobile_manipulation_;

  // Planning variables
  moveit::planning_interface::MoveGroupInterfacePtr whole_move_group_ptr_;
  moveit::planning_interface::MoveGroupInterfacePtr arm_move_group_ptr_;
  const robot_state::JointModelGroup* arm_joint_model_group_;
  const robot_state::JointModelGroup* whole_joint_model_group_;
  planning_scene::PlanningScenePtr scene_;
  nav_msgs::OccupancyGrid occupancy_map_;
  moveit_msgs::DisplayTrajectory arm_trajectory_;

  // Poses
  geometry_msgs::Pose end_effector_target_pose_;
  geometry_msgs::Pose pre_grasp_pose_;
  geometry_msgs::Pose2D current_base_pose_;

  /**
   * @brief commutation_ The computed commutation configuration
   */
  RobotPlacement::CommutationConfiguration commutation_;
  RobotPlacement::InverseMapType inverse_map_type_;

  /**
   * @brief ending_ Describes the state at which the mobile manipulation ended. Useful for displaying errors if
   * unsuccessful
   */
  EndingState ending_;

  std::vector<int> vrep_joint_handles_;
};


#endif  // MOBILE_MANIPULATION_H
