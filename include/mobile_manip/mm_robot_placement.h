#ifndef ROBOT_PLACEMENT_H
#define ROBOT_PLACEMENT_H

// BASIC
#include <chrono>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/OccupancyGrid.h>

// MOVEIT
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// BOOST
#include <boost/filesystem.hpp>

// MAP_CREATOR
#include <map_creator/hdf5_dataset.h>

// OPTIMIZATION
#include "mm_opt.h"

//* Class for Robot Placement
/**
 * @brief The RobotPlacement The main class added for the thesis that computes the best robot placement for
 * the given scene
 */
class RobotPlacement
{
public:
  /**
   * @brief RobotPlacement Constructs the base_placement object using the moveit names of the whole robot, mobile
   * platform, and manipulator
   * @param whole_robot_name Name of the robot group as used with moveit
   * @param patform_name Name of the mobile platform group as used with moveit
   * @param arm_name Name of the manipulator group as used in moveit
   */
  RobotPlacement(std::string whole_robot_name, std::string patform_name, std::string arm_name);
  ~RobotPlacement();

  /**
   * @brief SortedQualityMap This sorts the inverse spheres in decreasing order of their quality value
   */
  typedef std::multimap<double, std::vector<double>, std::greater<double> > SortedQualityMap;

  /// The main struct describing the commutation configuration ie the robot placement
  typedef struct
  {
    /// Whether the Commutation Configuration was found or not
    bool found;
    /// 2D pose of the mobile platform
    geometry_msgs::Pose2D pose;
    /// Arm zeroth link position in space
    geometry_msgs::Point arm_location;
    /// Remaining available options in the inverse quality map
    SortedQualityMap remaining_options;
  } CommutationConfiguration;

  /**
   * @brief The inverseMapType enum for the kinds of inverse quality maps that can be used
   */
  enum InverseMapType
  {
    REACHABILITY,
    PLANABILITY,
    FUSION,
  };

  /**
   * @brief loadInverseMap loads the chosen inverse map from file
   * @param inverse_map is the variable in which the map is loaded
   * @param map_type - reachability, planability or combined
   * @return true if the map is loaded successfully else false
   */
  bool loadInverseMap(SortedQualityMap& inverse_map, InverseMapType map_type);

  // Focus on this and this will tell you how to fill other functions
  /**
   * @brief determineCommutationConfiguration  Main function that is called to determine the commutation configuration
   * @param target Pose msg describing the target pose
   * @param current_base_pose The current pose2D of the mobile base
   * @param scene The Planning Scene for MoveIt!
   * @param occupancy_map The 2D occupancy grid of the robot
   * @param inverse_spheres The main input of the inverted reachability/planability/fusion quality map
   * @return The Commutation Configuration ie the pose of the robot at which it must switch from locomotion to
   * manipulation
   */
  CommutationConfiguration determineCommutationConfiguration(geometry_msgs::Pose& target,
                                                             geometry_msgs::Pose2D& current_base_pose,
                                                             planning_scene::PlanningScenePtr& scene,
                                                             nav_msgs::OccupancyGrid& occupancy_map,
                                                             SortedQualityMap& inverse_spheres);

  /**
   * @brief refineCommutationConfiguration  When the robot is not able to reach the target pose due to some reason, a
   * new base placment may be carried out using this function
   * @param target Pose msg describing the target pose
   * @param current_base_pose The current pose2D of the mobile base
   * @param scene The Planning Scene for MoveIt!
   * @param occupancy_map The 2D occupancy grid of the robot
   * @param remaining_solutions The remaining untested spheres/positions from the quality map used for determining the
   * commutation configuration
   * @return The Commutation Configuratino ie the pose of the robot at which it must switch from locomotion to
   * manipulation
   */
  CommutationConfiguration refineCommutationConfiguration(geometry_msgs::Pose& target,
                                                          geometry_msgs::Pose2D& current_base_pose,
                                                          planning_scene::PlanningScenePtr& scene,
                                                          nav_msgs::OccupancyGrid& occupancy_map,
                                                          SortedQualityMap remaining_solutions);

private:
  /**
   * @brief loadInverseReachabilityMap Load the inverse reachability map
   * @param inverse_reach_map The inverse reachability map as specified by inverse_reach_map_path_ is stored here
   * @return true if the map is successfully loaded else false
   */
  bool loadInverseReachabilityMap(SortedQualityMap& inverse_reach_map);

  /**
   * @brief loadInversePlanabilityMap Load the inverse planability map
   * @param inverse_plan_map The inverse planability map as specified by inverse_plan_map_path_ is stored here
   * @return true if the map is successfully loaded else false
   */
  bool loadInversePlanabilityMap(SortedQualityMap& inverse_plan_map);

  /**
   * @brief fuseMap Fuse the inverse reachability and inverse planability maps into single inverse fusion map
   * @param inverse_fusion_map The computed inverse fusion map is stored here
   * @return true if fusion is successful else false
   */
  bool fuseMap(SortedQualityMap& inverse_fusion_map);

  /**
   * @brief transformMapAtPose Transform the inverse map at from origin to the chosen target pose
   * @param inverse_map Original inverse map
   * @param target Pose at which the inverse Quality map is to be transformed to
   * @param inverse_map_tf Transformed inverse map
   */
  void transformMapAtPose(SortedQualityMap& inverse_map, geometry_msgs::Pose& target, SortedQualityMap& inverse_map_tf);

  /**
   * @brief getArmHeight Determine the height at which the inverse map must be sliced to limit the search to feasible
   * arm locations
   * @param arm_link0_name The name of the zeroth link of the arm in moveit urdf
   * @param scene The planning scene used to compute the output height
   * @param arm_height The output computed height
   */
  void getArmHeight(std::string arm_link0_name, planning_scene::PlanningScenePtr& scene, double& arm_height);

  /**
   * @brief sliceMapAtArmHeight Slice the 3D map based on the height +/- threshold
   * @param inverse_map Input 3D inverse quality map
   * @param height The height at which the map must be sliced
   * @param threshold The tolerance around the slicing height which is used to find the spheres within the sliced layer
   * @param inverse_map_sliced The output sliced Inverse Quality Map in 2D
   */
  void sliceMapAtArmHeight(SortedQualityMap& inverse_map, double& height, double& threshold,
                           SortedQualityMap& inverse_map_sliced);

  /**
   * @brief filter2DMap Filter the feasible commutation configurations based on 2D costmap
   * @param inverse_map_2D Input sliced 2D map for filtering
   * @param occupancy_map 2D occupancy grid of the scene used to filter feasible positions
   * @param commutations_filtered The output 2D set of feasible commutation configurations
   */
  void filter2DMap(SortedQualityMap& inverse_map_2D, nav_msgs::OccupancyGrid& occupancy_map,
                   SortedQualityMap& commutations_filtered);

  /**
   * @brief coordToMapCell Convert Map coordinate to cell coordinate as in costmap_2d
   * @param cx The x coordinate
   * @param cy The y coordinate
   * @param mx The number of cells in x
   * @param my The number of cells in y
   * @param occupancy_map
   * @return true if the map coordinate was valid and could be converted to cell coordinate
   */
  bool coordToMapCell(double cx, double cy, unsigned int& mx, unsigned int& my, nav_msgs::OccupancyGrid& occupancy_map);

  /**
   * @brief generateTestStateConfiguration Generates a robot state at for given arm location when the robot has an angle
   * to arm's base.
   * @param arm_location The x,y,z vector describing the test arm position around the target
   * @param angle The yaw of the zeroth link of arm (generally equals 0)
   * @param link0_to_base_tf The transformation from the link0 of the arm to base_link
   * @param joint_config The output joint config at the test location for the arm
   */
  void generateTestStateConfiguration(tf::Vector3& arm_location, double angle, Eigen::Affine3d& link0_to_base_tf,
                                      std::vector<double>& joint_config);

  /**
   * @brief optimizeBasePose  Optimize the yaw angle for the base about the chosen arm location
   * @return Optimized Pose2D for the base
   */
  geometry_msgs::Pose2D optimizeBasePose(void);

  // Visualization

  /**
   * @brief setCommutationColor Sets the color of the sphere to be sent for visualization in rviz
   * @param quality The quality value of the sphere that is used to determine its color
   * @param color Color with which the sphere will be visualized
   */
  void setCommutationColor(double quality, rviz_visual_tools::colors& color);

  /**
   * @brief visualizeCommutations To trigger visualize a set of inverse quality map in rviz
   * @param quality_map The map to visualized
   * @param viz Pointer to the MoveIt! Visual tools
   */
  void visualizeCommutations(SortedQualityMap& quality_map, moveit_visual_tools::MoveItVisualToolsPtr viz);

  /**
   * @brief inverse_reach_map_path_ The path to the inverse reachability map file
   */
  std::string inverse_reach_map_path_;
  /**
   * @brief inverse_plan_map_path_ The path to the inverse planability map file
   */
  std::string inverse_plan_map_path_;
  /**
   * @brief inverse_plan_map_data_path_ The path to the auxillary iverse planability map file that contains its data
   */
  std::string inverse_plan_map_data_path_;
  /**
   * @brief slice_threshold_ The threshold to be used while slicing the 3D inverse quality map into 2D
   */
  double slice_threshold_;  // @todo: set based on voxel resolution
                            /**
                             * @brief whole_robot_group_ The name of the moveit group describing the whole robot
                             */
  std::string whole_robot_group_;
  /**
   * @brief platform_group_ The name of the moveit group describing the mobile platform
   */
  std::string platform_group_;
  /**
   * @brief arm_group_ The name of the moveit group describing the kinematic arm
   */
  std::string arm_group_;
};

#endif  // ROBOT_PLACEMENT_H
