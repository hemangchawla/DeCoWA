
#include "../include/mobile_manip/mm_robot_placement.h"

/** This program computes the base placement integrated with the moveit planning scene and the 2d cost map
 * - Inputs: criterion, planning scene, robot model, end effector target pose
 * - Outputs: commutation configuration
 *
 * The IR/IP map location would loaded here based on the parameter in the base_placement config file
 */

/**
 * Procedure:
 * - The inverse map is loaded
 * - It is transformed based on the target pose
 * - The map is sliced at the height of the arm
 * - The highest quality position is selected
 * - The position is checked against the 2D map
 * - Then the arm configuration at the target pose is calculated
 * - It is checked against collision
 * - If collision free then the base pose is optimized
 * - Accordingly the arm configuration (the zeroth joint) is updated
 */

/// Used to set the group name variables
RobotPlacement::RobotPlacement(std::string whole_robot_name, std::string platform_name, std::string arm_name)
{
  whole_robot_group_ = whole_robot_name;
  platform_group_ = platform_name;
  arm_group_ = arm_name;
}

RobotPlacement::~RobotPlacement()
{
}

/// The colors are divided into 5 categories from RED (lowest quality) to PURPLE (best quality)
void RobotPlacement::setCommutationColor(double quality, rviz_visual_tools::colors& color)
{
  if (quality >= 0 && quality < 20)
  {
    color = rviz_visual_tools::RED;
  }
  else if (quality >= 20 && quality < 40)
  {
    color = rviz_visual_tools::ORANGE;
  }
  else if (quality >= 40 && quality < 60)
  {
    color = rviz_visual_tools::GREEN;
  }
  else if (quality >= 60 && quality < 80)
  {
    color = rviz_visual_tools::BLUE;
  }
  else if (quality >= 80 && quality <= 100)
  {
    color = rviz_visual_tools::PURPLE;
  }
}

/// Iterate over all spheres in the inverse map and add rviz spheres at their centers with colors based on their quality
/// value
void RobotPlacement::visualizeCommutations(SortedQualityMap& quality_map, moveit_visual_tools::MoveItVisualToolsPtr viz)
{
  Eigen::Vector3d center;

  for (SortedQualityMap::iterator it = quality_map.begin(); it != quality_map.end(); ++it)
  {
    center[0] = it->second[0];
    center[1] = it->second[1];
    center[2] = it->second[2];
    rviz_visual_tools::colors color;
    setCommutationColor(it->first, color);
    viz->publishSphere(center, color, rviz_visual_tools::XXLARGE);
  }
  viz->trigger();
}

void RobotPlacement::generateTestStateConfiguration(tf::Vector3& arm_location, double angle,
                                                    Eigen::Affine3d& link0_to_base_tf,
                                                    std::vector<double>& joint_config)
{
  // Make a frame
  tf::Transform test_arm_tf;
  tf::Quaternion arm_quat;
  arm_quat.setEuler(0, 0, angle);  // So angle is wrt the link0 angle
  test_arm_tf.setOrigin(arm_location);
  test_arm_tf.setRotation(arm_quat);

  Eigen::Affine3d test_arm_frame;
  tf::transformTFToEigen(test_arm_tf, test_arm_frame);

  Eigen::Affine3d test_base_frame = test_arm_frame * link0_to_base_tf;

  tf::Transform test_base_location;
  tf::transformEigenToTF(test_base_frame, test_base_location);

  // Move the base pose to the correponding location (since arm cannot be move alone)
  joint_config[0] = test_base_location.getOrigin().getX();
  joint_config[1] = test_base_location.getOrigin().getY();
  joint_config[2] = tf::getYaw(test_base_location.getRotation());
  // Rest of them remain the same as the test state, i.e. zero.
}

bool RobotPlacement::coordToMapCell(double cx, double cy, unsigned int& mx, unsigned int& my,
                                    nav_msgs::OccupancyGrid& occupancy_map)
{
  double origin_x = occupancy_map.info.origin.position.x;
  double origin_y = occupancy_map.info.origin.position.y;
  double resolution = occupancy_map.info.resolution;
  double size_x = occupancy_map.info.width;
  double size_y = occupancy_map.info.height;

  if (cx < origin_x || cy < origin_y)
  {
    return false;
  }
  mx = (int)((cx - origin_x) / resolution);
  my = (int)((cy - origin_y) / resolution);

  if (mx < size_x && my < size_y)
  {
    return true;
  }
  return false;
}

void RobotPlacement::filter2DMap(SortedQualityMap& inverse_map_2D, nav_msgs::OccupancyGrid& occupancy_map,
                                 SortedQualityMap& commutations_filtered)
{
  // Translate the coordinate to the relevant cell (e.g. using the worldToMap function), check the costs (e.g. using the
  // getCost function) and then compare against the cost values).
  // or write your own world to map function
  // use that to determine the cost
  /// Filter those that have cost > 0
  commutations_filtered.clear();

  for (SortedQualityMap::iterator it = inverse_map_2D.begin(); it != inverse_map_2D.end(); ++it)
  {
    // Compute corresponding cell in map
    // NOTE: The input coorinates must also be in the same frame as the map frame. Else they need to be converted first.
    unsigned int mx;
    unsigned int my;
    bool cell;
    cell = coordToMapCell(it->second[0], it->second[1], mx, my, occupancy_map);
    unsigned int index = my * occupancy_map.info.width + mx;
    char val = occupancy_map.data[index];

    if ((!cell) || (val == 0) ||
        (val == -1))  // The coordinate is unknown in map (hence not a known obstacle) or is free
    {
      commutations_filtered.insert(std::make_pair(it->first, it->second));
    }
  }
}

void RobotPlacement::getArmHeight(std::string arm_link0_name, planning_scene::PlanningScenePtr& scene,
                                  double& arm_height)
{
  robot_state::RobotState state = scene->getCurrentStateNonConst();
  Eigen::Affine3d link0_wrt_odom = state.getGlobalLinkTransform(arm_link0_name);
  // The third element represents the z axis i.e. height
  arm_height = link0_wrt_odom.translation()[2];
}

void RobotPlacement::sliceMapAtArmHeight(SortedQualityMap& inverse_map, double& height, double& threshold,
                                         SortedQualityMap& inverse_map_sliced)
{
  inverse_map_sliced.clear();
  std::cout << "thresh: " << threshold << '\n';

  for (SortedQualityMap::iterator it = inverse_map.begin(); it != inverse_map.end(); ++it)
  {
    if (it->second[2] > (height - threshold) && it->second[2] < (height + threshold))
    {
      inverse_map_sliced.insert(std::make_pair(it->first, it->second));
    }
  }
}

void RobotPlacement::transformMapAtPose(SortedQualityMap& inverse_map, geometry_msgs::Pose& target,
                                        SortedQualityMap& inverse_map_tf)
{
  /// @@todo point transformation of map (currently pose)

  inverse_map_tf.clear();

  for (SortedQualityMap::iterator it = inverse_map.begin(); it != inverse_map.end(); ++it)
  {
    tf2::Vector3 target_translation(target.position.x, target.position.y, target.position.z);
    tf2::Quaternion target_rotation(target.orientation.x, target.orientation.y, target.orientation.z,
                                    target.orientation.w);
    target_rotation.normalize();

    tf2::Transform target_transform;
    target_transform.setOrigin(target_translation);
    target_transform.setRotation(target_rotation);

    tf2::Vector3 inverse_sphere_position(it->second[0], it->second[1], it->second[2]);
    tf2::Vector3 inverse_sphere_position_tf = target_transform * inverse_sphere_position;
    std::vector<double> new_inverse_sphere_position;
    new_inverse_sphere_position.push_back(inverse_sphere_position_tf.x());
    new_inverse_sphere_position.push_back(inverse_sphere_position_tf.y());
    new_inverse_sphere_position.push_back(inverse_sphere_position_tf.z());

    inverse_map_tf.insert(std::make_pair(it->first, new_inverse_sphere_position));
  }
}

bool RobotPlacement::fuseMap(SortedQualityMap& inverse_fusion_map)
{
  ROS_INFO("Fuse map");
  MultiMap reach_pose_col_filter;
  MapVecDouble reach_sp;
  float reach_res;

  // Load reachability map
  if (ros::param::get("/inverse_reach_map_path", inverse_reach_map_path_))
  {
    if (!boost::filesystem::exists(inverse_reach_map_path_))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      hdf5_dataset::Hdf5Dataset h5file(inverse_reach_map_path_);
      h5file.open();
      h5file.loadMapsFromDataset(reach_pose_col_filter, reach_sp, reach_res);
    }
  }
  else
  {
    ROS_INFO("Please specify reachability and planability maps path");
    return false;
  }

  MultiMap plan_pose_col_filter;
  MapVecDouble plan_sp;
  float plan_res;

  // Load planability map
  if (ros::param::get("/inverse_plan_map_data_path", inverse_plan_map_data_path_))
  {
    if (!boost::filesystem::exists(inverse_plan_map_data_path_))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      hdf5_dataset::Hdf5Dataset h5file(inverse_plan_map_data_path_);
      h5file.open();
      h5file.loadMapsFromDataset(plan_pose_col_filter, plan_sp, plan_res);
      slice_threshold_ = plan_res / 2.0;
    }
  }
  else
  {
    ROS_INFO("Please specify reachability and planability maps path");
    return false;
  }

  // Fusion happens here

  MapVecDouble fusion_sp;
  for (auto it = plan_sp.begin(); it != plan_sp.end(); ++it)
  {
    double reach = reach_sp.find(it->first)->second;  // Reachability quality
    double planning_cost = it->second;                // Planning cost
    double quality = 0;
    if (planning_cost > 0)
    {
      quality = double(reach) / planning_cost;
    }

    fusion_sp.insert(std::make_pair(it->first, quality));
  }

  // Sorth the spheres based on quality
  for (MapVecDouble::iterator it = fusion_sp.begin(); it != fusion_sp.end(); ++it)
  {
    std::vector<double> sphere_coord(3);
    sphere_coord[0] = it->first[0];
    sphere_coord[1] = it->first[1];
    sphere_coord[2] = it->first[2];
    inverse_fusion_map.insert(std::make_pair(it->second, sphere_coord));
  }
  return true;
}

bool RobotPlacement::loadInverseReachabilityMap(SortedQualityMap& inverse_reach_map)
{
  inverse_reach_map.clear();

  // The parameter storing the path to IRM is used to load the file.
  if (ros::param::get("/inverse_reach_map_path", inverse_reach_map_path_))
  {
    MultiMap pose_col_filter;
    MapVecDouble sp;
    float res;

    if (!boost::filesystem::exists(inverse_reach_map_path_))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      hdf5_dataset::Hdf5Dataset h5file(inverse_reach_map_path_);
      h5file.open();
      h5file.loadMapsFromDataset(pose_col_filter, sp, res);

      slice_threshold_ = res / 2.0;

      // creating the column of spheres and respective reach-probaility values
      // The reachability values are the keys as the sorting is done wrt the keys
      for (MapVecDouble::iterator it = sp.begin(); it != sp.end(); ++it)
      {
        std::vector<double> sphere_coord(3);
        sphere_coord[0] = it->first[0];
        sphere_coord[1] = it->first[1];
        sphere_coord[2] = it->first[2];
        inverse_reach_map.insert(std::make_pair(it->second, sphere_coord));
      }
      return true;
    }
  }
  else
  {
    ROS_ERROR("Inverse reachability map not found! Check path to file");
    return false;
  }
}

bool RobotPlacement::loadInversePlanabilityMap(SortedQualityMap& inverse_plan_map)
{
  inverse_plan_map.clear();

  // The parameter storing the path to IRM is used to load the file.
  if (ros::param::get("/inverse_plan_map_path", inverse_plan_map_path_))
  {
    MultiMap pose_col_filter;
    MapVecDouble sp;
    float res;

    if (!boost::filesystem::exists(inverse_plan_map_path_))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      hdf5_dataset::Hdf5Dataset h5file(inverse_plan_map_path_);
      h5file.open();
      h5file.loadMapsFromDataset(pose_col_filter, sp, res);

      slice_threshold_ = res / 2.0;

      // Creating the column of spheres and respective reach-probaility values
      // The reachability values are the keys as the sorting is done wrt the keys
      for (MapVecDouble::iterator it = sp.begin(); it != sp.end(); ++it)
      {
        std::vector<double> sphere_coord(3);
        sphere_coord[0] = it->first[0];
        sphere_coord[1] = it->first[1];
        sphere_coord[2] = it->first[2];
        inverse_plan_map.insert(std::make_pair(it->second, sphere_coord));
      }
      return true;
    }
  }
  else
  {
    ROS_ERROR("Inverse planability map not found! Check path to file");
    return false;
  }
}

bool RobotPlacement::loadInverseMap(SortedQualityMap& inverse_map, InverseMapType map_type)
{
  if (map_type == RobotPlacement::REACHABILITY)
  {
    ROS_INFO("Loading Inverse Reachability map");
    bool loaded = loadInverseReachabilityMap(inverse_map);

    if (!loaded)
    {
      return false;
    }
    return true;
  }
  else if (map_type == RobotPlacement::PLANABILITY)
  {
    ROS_INFO("Loading Inverse Planability map");
    bool loaded = loadInversePlanabilityMap(inverse_map);

    if (!loaded)
    {
      return false;
    }
    return true;
  }
  else if (map_type == RobotPlacement::FUSION)
  {
    ROS_INFO("Loading Inverse Fusion map");
    bool loaded = fuseMap(inverse_map);

    if (!loaded)
    {
      return false;
    }
    return true;
  }
  else
  {
    ROS_ERROR("Invalid Map Type");
    return false;
  }
}

RobotPlacement::CommutationConfiguration RobotPlacement::determineCommutationConfiguration(
    geometry_msgs::Pose& target, geometry_msgs::Pose2D& current_base_pose, planning_scene::PlanningScenePtr& scene,
    nav_msgs::OccupancyGrid& occupancy_map, SortedQualityMap& inverse_spheres)
{
  // Start clock to measure the time it takes to compute the robot placement
  std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

  // Visualization
  std::string marker_topic = "markers";
  std::string robot_state_topic = "robot_state";
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom", marker_topic));
  visual_tools_->deleteAllMarkers();
  visual_tools_->loadMarkerPub();
  visual_tools_->loadRobotStatePub(robot_state_topic);

  visual_tools_->publishSphere(target, rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  visual_tools_->trigger();

  // Initialize solution variable
  RobotPlacement::CommutationConfiguration commutation;

  // Initialize joint group models
  const robot_model::RobotModelConstPtr kinematic_model = scene->getRobotModel();
  const robot_state::JointModelGroup* whole_joint_model_group = kinematic_model->getJointModelGroup(whole_robot_group_);
  const robot_state::JointModelGroup* arm_joint_model_group = kinematic_model->getJointModelGroup(arm_group_);

  // Initialize collision detection
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.distance = false;
  collision_request.cost = false;

  // @todo use name of arm kin chain as param
  // arm_joint_model group only used for IK. Rest everything using the whole_joint_model_group

  // Check if reachable and collision free from current pose
  robot_state::RobotState& test_state = scene->getCurrentStateNonConst();

  // @todo replace with collision aware IK (with separeate validity functions for arm and base)
  bool reach_target = test_state.setFromIK(arm_joint_model_group, target);

  if (reach_target)
  {
    collision_request.group_name = whole_robot_group_;
    collision_result.clear();
    test_state.updateCollisionBodyTransforms();
    scene->checkCollision(collision_request, collision_result, test_state, scene->getAllowedCollisionMatrix());

    if (!collision_result.collision)
    {
      commutation.found = true;
      commutation.pose = current_base_pose;
      return commutation;
    }
  }

  // Transform Map
  SortedQualityMap inverse_spheres_tf;
  transformMapAtPose(inverse_spheres, target, inverse_spheres_tf);
  ROS_INFO_STREAM("Transformed. Number of voxels:" << inverse_spheres_tf.size());

  // Slice Map
  SortedQualityMap inverse_spheres_tf_sliced;
  double arm_height;
  getArmHeight("link0", scene, arm_height);  // @todo get arm base from user or other means
  sliceMapAtArmHeight(inverse_spheres_tf, arm_height, slice_threshold_, inverse_spheres_tf_sliced);
  ROS_INFO_STREAM("Sliced. Number of voxels: " << inverse_spheres_tf_sliced.size());

  // Filter Map
  SortedQualityMap commutations_filtered;
  filter2DMap(inverse_spheres_tf_sliced, occupancy_map, commutations_filtered);
  ROS_INFO_STREAM("Filtered. Nomber of voxels: " << commutations_filtered.size());

  // Visualize commutations
  visualizeCommutations(commutations_filtered, visual_tools_);

  // Test state that will be used to compute the commutation configuration
  test_state.setToDefaultValues(whole_joint_model_group, "base_arm_zero");  // @todo Default config name
  std::vector<double> test_joint_group_positions;
  test_state.copyJointGroupPositions(whole_joint_model_group, test_joint_group_positions);  // Is this line needed?

  // Compute constant transform between link0 of arm and base_link of platform
  Eigen::Affine3d odom_to_base_link = test_state.getGlobalLinkTransform("base_link");
  Eigen::Affine3d odom_to_link0 = test_state.getGlobalLinkTransform("link0");
  Eigen::Affine3d link0_to_base_link = odom_to_link0.inverse() * odom_to_base_link;

  // Main loop for finding the best commutation configuration iterating over the remaining options in decreasing order
  // of quality
  for (SortedQualityMap::iterator it = commutations_filtered.begin(); it != commutations_filtered.end(); ++it)
  {
    // GENERATE ARM CONFIGURATION
    // Generate test state based on arm location
    tf::Vector3 test_arm_location(it->second[0], it->second[1], it->second[2]);
    generateTestStateConfiguration(test_arm_location, 0, link0_to_base_link, test_joint_group_positions);
    test_state.setJointGroupPositions(whole_joint_model_group, test_joint_group_positions);

    // Compute IK (Only the arm)
    bool set_config = test_state.setFromIK(arm_joint_model_group, target);

    // If IK found
    if (set_config)
    {
      // Collision checking of arm with world
      collision_request.group_name = arm_group_;
      collision_result.clear();
      test_state.updateCollisionBodyTransforms();
      scene->getCollisionWorld()->checkRobotCollision(collision_request, collision_result,
                                                      *(scene->getCollisionRobot()), test_state,
                                                      scene->getAllowedCollisionMatrix());

      // If arm is collision free at  location
      if (!collision_result.collision)
      {
        // Generate Base Orientation
        // Optimize Base Location
        double base_angle =
            mm_opt::solve(test_arm_location, scene, link0_to_base_link, platform_group_, current_base_pose);

        if (!std::isnan(base_angle))
        {
          // Generate complete robot configuration
          generateTestStateConfiguration(test_arm_location, base_angle, link0_to_base_link, test_joint_group_positions);
          test_state.setJointGroupPositions(whole_joint_model_group, test_joint_group_positions);
          // Compute IK (This is whole body)
          test_state.setFromIK(arm_joint_model_group, target);

          // Check whole body collision
          collision_request.group_name = whole_robot_group_;
          collision_result.clear();
          test_state.updateCollisionBodyTransforms();
          scene->checkCollision(collision_request, collision_result, test_state, scene->getAllowedCollisionMatrix());

          if (!collision_result.collision)
          {
            visual_tools_->publishRobotState(test_state);
            visual_tools_->trigger();

            commutation.found = true;
            commutation.pose.x = test_joint_group_positions[0];
            commutation.pose.y = test_joint_group_positions[1];
            commutation.pose.theta = test_joint_group_positions[2];
            commutation.remaining_options = commutations_filtered;

            commutation.arm_location.x = test_arm_location[0];
            commutation.arm_location.y = test_arm_location[1];
            commutation.arm_location.z = test_arm_location[2];
            std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
            std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
            ROS_INFO_STREAM("Time requierd to determine commutation (without loading): " << ms.count() << "ms");

            return commutation;
          }
          else
          {
            commutations_filtered.erase(it);
          }
        }
        else
        {
          commutations_filtered.erase(it);
        }
      }
      else
      {
        commutations_filtered.erase(it);
      }
    }
    else
    {
      commutations_filtered.erase(it);
    }
  }
  commutation.found = false;
  std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
  std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
  ROS_INFO_STREAM("Time requierd to determine commutation (without loading): " << ms.count() << "ms");

  return commutation;
}

// Reasoning for using Octomap and Costmap2d both:

// The optimization must use the moveit collision checking as the one in 2d map does not use
// orietations and the circumscribed cost will mostly always be there.
// If we use that then the cell with zero is preferable. However since we are near table or something.
// Such a cell may not be possible. Hence, the costs would be circumscribed "possible collision"
// Now if a zero cell is found, great! But since that is unlikely we will be selecting between the
// possibly in collision cells.
// However it is not helpful as we want to as much sure as we possibly can be about the collision
// freeness of the location.
// On the contrary only using the 3d map implies that areas that can be seen through the laser scanner
// are missed and selected as free. If such a location is output, then the result is unusable.
// Hence the location must be free or possibly circumscribed. If it is known to be in unavailable,
// then it must be rejected.
