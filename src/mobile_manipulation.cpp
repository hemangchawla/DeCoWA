#include "../include/mobile_manip/mobile_manipulation.h"

/**
 * In this main node the state machine / conceptual framework of mobile manipulation is implemented.
 * This is the node, user needs to launch.
 * There are  hence 2 components.
 * 1) Base Placement Module.
 * 2) Motion Planning Module.
 *
 * The Base placement module generates the best 3D pose (x, y , theta) to be reached by the robot base in order to grab
 * an object.
 * The Motion Planning is divided into two parts for the sequential system: for the base, and for the arm.
 *
 * The base and the arm planning must again be classified whether it is planning in vrep or in real world.
 *
 * It can be assume that a pose in the environment needs to be reached.
 *
 * It can be replaced by position, or a region of positions, or a region of poses.
 *
 * So the EE pose replaced by position, or a region of positions, or a region of poses.
 * assume that a pose in the environment needs to be reached.
 *
 * It can be replaced by position, or a region of positions, or a region of poses.
 *
 * Files:
 * 1. Main: where the end effector pose is read and the state machine is controlled.
 * The moveit and navigation stack basics must also be initiated here.
 * Question: How is the end effector pose generated in real life?
 * 2.Base Placement File : Using end effector target and the 2D map and the 3D map, determine the x,y, theta coordinates
 * wrt current pose as the goal for the mobile base.
 * 3. Arm motion planning
 * 4. Base motion planning
 * 5. Others: Reachability map generation, Inverse Reachability map, Planability map, Inverse Planability Map
 */

MobileManipulation::MobileManipulation()
{
  // Init booleans
  end_effector_target_pose_rcvd_ = false;
  computing_base_motion_successful_ = false;
  executing_base_motion_successful_ = false;
  pre_grasp_pose_rcvd_ = false;
  unfolding_arm_successful_ = false;
  computing_arm_motion_successful_ = false;
  executing_arm_motion_successful_ = false;
  pre_grasp_pose_rcvd_ = false;

  vrep_joint_handles_ = { 0 };

  // Load parameters from server
  loadParameters();

  // END EFFECTOR TARGET
  // The input is read from a topic
  // The subscrber will depend on whether the mode is vrep or real robot.
  if (!simulation_mode_)
  {
    Subscriber_end_effector_target_ =
        nh_.subscribe("/mobile_manipulation/goal_ee", 1, &MobileManipulation::setEndEffectorTarget, this);
  }
  else
  {
    Subscriber_vrep_joint_handles_ =
        nh_.subscribe("/vrep/joint_handles", 1, &MobileManipulation::getVrepJointHandles, this);
    Subscriber_end_effector_target_ =
        nh_.subscribe("/vrep/goal_ee", 1, &MobileManipulation::setEndEffectorTarget, this);

    Subscriber_odom_ = nh_.subscribe("/vrep/pose", 1, &MobileManipulation::getCurrentPlatformPose, this);

    // 2D MAP
    // Subscribe to 2D occupancy grid message
    Subscriber_2D_map_ = nh_.subscribe("/projected_map", 1, &MobileManipulation::get2DMap, this);
  }

  Subscriber_arm_path_ =
      nh_.subscribe("/move_group/display_planned_path", 10, &MobileManipulation::readArmTrajectory, this);
}

MobileManipulation::~MobileManipulation()
{
}
/// The execution mode, the inverse map type and the move group names are set
void MobileManipulation::loadParameters()
{
  // EXECUTION MODE
  // This param is used to identify is the simulation mode is on.
  setExecutionMode();

  // INVERSE_MAP_TYPE
  // The parameter that decides whether the reachability, planability, or fusion map must be used is read.
  setInverseMapType();

  // MOVEIT Groups
  setMoveGroupNames();
}

void MobileManipulation::getVrepJointHandles(const std_msgs::Int32MultiArray& msg)
{
  vrep_joint_handles_.clear();
  for (int i = 0; i < msg.data.size(); ++i)
  {
    vrep_joint_handles_.push_back(msg.data[i]);
  }
  ROS_INFO("Received joint Handles from vrep!");
  // Now we shutdown the subscriber as these won't change
  Subscriber_vrep_joint_handles_.shutdown();
}

void MobileManipulation::setMoveGroupNames()
{
  if ((nh_.getParam("/platform_group", platform_group_)) && (nh_.getParam("/arm_group", arm_group_)) &&
      (nh_.getParam("/whole_body_group", whole_robot_group_)))
  {
    ROS_INFO_STREAM("Whole Body Group: " << whole_robot_group_ << ", Platform Group: " << platform_group_
                                         << ", Arm Group: " << arm_group_);
  }
  else
  {
    ROS_ERROR("One of the platform, arm, or whole body movegroups not specified! Shutting Down");
    ros::shutdown();
  }

  // NAME OF ARM's BASE
  if (nh_.getParam("/arm_link0_name", arm_link0_name_))
  {
    ROS_INFO_STREAM("Arm's base name is:" << arm_link0_name_);
  }
  else
  {
    ROS_WARN("Arm base name not found. Initializing config to zero");
  }
}

void MobileManipulation::setExecutionMode()
{
  if (nh_.getParam("/execution_mode", execution_mode_))
  {
    if (execution_mode_.compare("simulation") == 0)
    {
      ROS_INFO("Simulation Mode");
      simulation_mode_ = true;
    }
    else if (execution_mode_.compare("real") == 0)
    {
      ROS_INFO("Real Mode");
      simulation_mode_ = false;
    }
    else
    {
      ROS_WARN("Execution mode not correct! Choose between simulation and real. Defaulting to simulation mode!");
      simulation_mode_ = true;
    }
  }
  else
  {
    ROS_ERROR("Execution mode: simulation/real not specified. Shutting down!");
    ros::shutdown();
  }
}

void MobileManipulation::setInverseMapType()
{
  if (nh_.getParam("/inverse_map_type", map_type_))
  {
    if (map_type_.compare("reachability") == 0)
    {
      ROS_INFO("Using Inverse Reachability Map");
      inverse_map_type_ = RobotPlacement::REACHABILITY;
    }
    else if (map_type_.compare("planability") == 0)
    {
      ROS_INFO("Using Inverse Planability Map");
      inverse_map_type_ = RobotPlacement::PLANABILITY;
    }
    else if (map_type_.compare("fusion") == 0)
    {
      ROS_INFO("Using fusion of Inverse Reachability and Planability Maps");
      inverse_map_type_ = RobotPlacement::FUSION;
    }
    else
    {
      ROS_WARN("Incorrect Map Type provided, will default to reachability map");
      inverse_map_type_ = RobotPlacement::REACHABILITY;
    }
  }
  else
  {
    ROS_ERROR("Map type not provided! Shutting down!");
    ros::shutdown();
  }
}

void MobileManipulation::get2DMap(const nav_msgs::OccupancyGrid& msg)
{
  // Temporarily store the map.
  occupancy_map_.header = msg.header;
  occupancy_map_.info = msg.info;
  occupancy_map_.data = msg.data;
}

void MobileManipulation::setEndEffectorTarget(const geometry_msgs::Pose msg)
{
  state_mobile_manipulation_ = COMPUTING_TARGET;
  end_effector_target_pose_ = msg;
  end_effector_target_pose_rcvd_ = true;
}

void MobileManipulation::getCurrentPlatformPose(const geometry_msgs::Pose2D& msg)
{
  current_base_pose_ = msg;
}

void MobileManipulation::readArmTrajectory(const moveit_msgs::DisplayTrajectory& msg)
{
  ROS_INFO_STREAM("Trajectory received with " << msg.trajectory[0].joint_trajectory.points.size() << " points");
  arm_trajectory_ = msg;
}

void MobileManipulation::stateMachineMobileManipulation()
{
  switch (state_mobile_manipulation_)
  {
    case IDLE:
    {
      ROS_INFO("Idle! Awaiting Target");

      // TODO: Add timer after which the program ends
      if (end_effector_target_pose_rcvd_)
      {
        state_mobile_manipulation_ = COMPUTING_BASE_PLACEMENT;
        ROS_INFO("Target received");
      }
      break;
    }
    case COMPUTING_BASE_PLACEMENT:
    {
      ROS_INFO("Computing Base Placement");

      RobotPlacement place_base(whole_robot_group_, platform_group_, arm_group_);

      // Load the inverse map type
      RobotPlacement::SortedQualityMap inverse_spheres;
      bool map_loaded = place_base.loadInverseMap(inverse_spheres, inverse_map_type_);

      if (!map_loaded)
      {
        ROS_ERROR("Inverse Map not loaded");
        ending_ = COMPUTING_COMMUTATION_CONFIGURATION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
        return;
      }
      commutation_ = place_base.determineCommutationConfiguration(end_effector_target_pose_, current_base_pose_, scene_,
                                                                  occupancy_map_, inverse_spheres);

      if (!commutation_.found)
      {  // if the comutation of commutation configuration is unsuccessful
        ending_ = COMPUTING_COMMUTATION_CONFIGURATION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      else if ((commutation_.pose.x == current_base_pose_.x) && (commutation_.pose.y == current_base_pose_.y) &&
               (commutation_.pose.theta == current_base_pose_.theta))
      {
        // If the object is reachable from current position
        state_mobile_manipulation_ = COMPUTING_PRE_GRASP_POSE;
      }
      else
      {
        state_mobile_manipulation_ = COMPLETE;
        ros::shutdown();
      }
      break;
    }

    case COMPUTING_BASE_MOTION:
    {
      // send base goal to Navigation stack or parabolic planner (from ros param)
      // receive bool success or failure
      // if success move to executingBaseMotion else move to complete

      // TODO: Read tolerance from user
      computing_base_motion_successful_ = mm_locomtion::computePath(current_base_pose_, commutation_.pose, 0.1);

      if (computing_base_motion_successful_)
      {
        state_mobile_manipulation_ = EXECUTING_BASE_MOTION;
      }
      else
      {
        ending_ = COMPUTING_BASE_MOTION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      break;
    }
    case EXECUTING_BASE_MOTION:
    {
      executing_base_motion_successful_ = mm_locomtion::locomote(commutation_.pose);
      if (executing_base_motion_successful_)
      {
        state_mobile_manipulation_ = COMPUTING_PRE_GRASP_POSE;
      }
      else
      {
        ending_ = EXECUTING_BASE_MOTION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      break;
    }
    case COMPUTING_PRE_GRASP_POSE:
    {
      ROS_INFO("Computing Pre grasp pose");

      /// Currently assume the pre-grasp pose is known (sets the same as end effector target)
      // TODO: Replace this with genetic search approach
      pre_grasp_pose_ = end_effector_target_pose_;
      pre_grasp_pose_rcvd_ = true;

      // if found move to computing arm motion else move to complete
      if (pre_grasp_pose_rcvd_)
      {
        state_mobile_manipulation_ = COMPUTING_ARM_MOTION;
      }
      else
      {
        ending_ = PRE_GRASP_COMPUTATION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      break;
    }
    case COMPUTING_ARM_MOTION:
    {
      // if motion plan found move to unfoldingArm
      // else move to complete

      robot_state::RobotState unfolded_state = scene_->getCurrentStateNonConst();
      if (unfolded_state_name_.length() != 0)
      {
        unfolded_state.setToDefaultValues(arm_joint_model_group_, unfolded_state_name_);
      }
      else
      {
        ROS_WARN_STREAM("No unfolded state " << unfolded_state_name_
                                             << " found. Arm will not be unfolded before motion "
                                                "planning");
      }

      ROS_INFO("Computing Motion Plan for manipulator");
      computing_arm_motion_successful_ =
          mm_manipulation::computeMotionPlan(arm_move_group_ptr_, unfolded_state, end_effector_target_pose_);

      if (computing_arm_motion_successful_)
      {
        state_mobile_manipulation_ = (unfolded_state_name_.length() != 0) ? UNFOLDING_ARM : EXECUTING_ARM_MOTION;
      }
      else
      {
        ending_ = COMPUTING_ARM_MOTION_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      break;
    }
    case UNFOLDING_ARM:
    {
      // await completing of unfolding
      // This will be a function in the arm file
      // if successful move to executing arm motion
      // else move to complete

      ROS_INFO("Unfolding arm");
      std::vector<double> folded_positions;
      std::vector<double> unfolded_positions;

      // Here the states have to reflect the arm positions only.
      robot_state::RobotState folded_state = scene_->getCurrentStateNonConst();
      robot_state::RobotState unfolded_state = scene_->getCurrentStateNonConst();

      if (zero_state_name_.length() != 0)
      {
        folded_state.setToDefaultValues(arm_joint_model_group_, zero_state_name_);
      }
      else
      {
        ROS_WARN("Home state/Zero state not specified. Assuming current state to be zero");
      }

      unfolded_state.setToDefaultValues(arm_joint_model_group_, unfolded_state_name_);

      folded_state.copyJointGroupPositions(arm_joint_model_group_, folded_positions);
      unfolded_state.copyJointGroupPositions(arm_joint_model_group_, unfolded_positions);

      std::vector<std::string> joint_names = arm_joint_model_group_->getJointModelNames();

      unfolding_arm_successful_ = mm_manipulation::unfoldArm(folded_positions, unfolded_positions, joint_names,
                                                                 simulation_mode_, vrep_joint_handles_);

      if (unfolding_arm_successful_)
      {
        state_mobile_manipulation_ = EXECUTING_ARM_MOTION;
      }
      else
      {
        ending_ = UNFOLDING_ARM_FAIL;
        state_mobile_manipulation_ = COMPLETE;
      }
      break;
    }
    case EXECUTING_ARM_MOTION:
    {
      // Execute arm motion,
      // if successful move to idle,
      // if failed move to idle

      ROS_INFO("Executing Arm Motion");

      executing_arm_motion_successful_ =
          mm_manipulation::executeMotion(arm_trajectory_, simulation_mode_, SPIN_RATE, vrep_joint_handles_);

      if (executing_arm_motion_successful_)
      {
        ending_ = SUCCESS;
      }
      else
      {
        ending_ = EXECUTING_ARM_MOTION_FAIL;
      }
      state_mobile_manipulation_ = COMPLETE;
      break;
    }
    case COMPLETE:
    {
      switch (ending_)
      {
        case SUCCESS:
        {
          ROS_INFO("Successfully reached target pose!");
          // Return success
          break;
        }
        case TARGET_NOT_RECEIVED:
        {
          ROS_WARN("Target Not Received!");
          // Ask if you want to pick object or cancel
          break;
        }
        case COMPUTING_COMMUTATION_CONFIGURATION_FAIL:
        {
          ROS_WARN("Could not find commutation configuration!");
          ROS_INFO("Shutting Down!");
          ros::shutdown();
          break;
        }
        case COMPUTING_BASE_MOTION_FAIL:
        {
          ROS_WARN("Could not compute path for the base!");
          // The commutation configuration is not reachable. Some other commutation configuration must be chosen.
          break;
        }
        case EXECUTING_BASE_MOTION_FAIL:
        {
          ROS_WARN("Unable to follow base path! Reaching commutation configuraion failed");
          // Was unable to reach commutation configuration
          // 1. Try again
          // 2. Choose another commutation configuration
          // 3. Give up
          break;
        }
        case PRE_GRASP_COMPUTATION_FAIL:
        {
          ROS_WARN("Cannot determine pre-grasp pose!");
          // Options:
          // 1. Give up
          // 2. Choose original traget pose as pre-graasp pose
          // 3. Move base to another location where the IK solution may be found to pre-grasp pose
          break;
        }
        case UNFOLDING_ARM_FAIL:
        {
          // Hardware error! Return Error
          ROS_ERROR("Hardware error! Cannot unfold arm!");
          // Shutdown node
          ROS_INFO("Shutting down node!");
          ros::shutdown();
          break;
        }
        case COMPUTING_ARM_MOTION_FAIL:
        {
          ROS_WARN("Cannot compute arm motion plan!");
          ros::shutdown();
          // Move to another location? Or return error?
          // Use another planner?
          break;
        }
        case EXECUTING_ARM_MOTION_FAIL:
        {
          // Currently the only way arm motion can fail is due to hardware error. Feedback is not used.
          // Hence display error
          ROS_ERROR("Hardware Error! Cannot execute arm motion plan!");
          // Shutdown node
          ROS_INFO("Shutting down node!");
          ros::shutdown();
          // TODO: Integrate with feedback control and identify reason for failure
          break;
        }
        default:
        {
          ROS_ERROR("Base Placement ended in unknown state!");
          ROS_INFO("Shutting Down!");
          ros::shutdown();
          break;
        }
      }
      ros::shutdown();
      break;
    }
    default:
    {
      ROS_WARN("State unknown!");
      break;
    }
  }
  return;
}

void MobileManipulation::spin()
{
  ROS_INFO("spin");
  ros::Rate rate(SPIN_RATE);

  // Initiating MoveIt!
  // Groups
  whole_move_group_ptr_.reset(new moveit::planning_interface::MoveGroupInterface(whole_robot_group_));
  arm_move_group_ptr_.reset(new moveit::planning_interface::MoveGroupInterface(arm_group_));
  whole_joint_model_group_ = whole_move_group_ptr_->getCurrentState()->getJointModelGroup(whole_robot_group_);
  arm_joint_model_group_ = arm_move_group_ptr_->getCurrentState()->getJointModelGroup(arm_group_);
  //  whole_move_group_ptr_->startStateMonitor();

  // Initiating and connecting to MoveIt! planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
  planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
  planning_scene_monitor::LockedPlanningSceneRW psrw =
      planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
  scene_ = psrw;

  // Read Unfolding state (if exists)
  nh_.getParam("/unfolded_state", unfolded_state_name_);
  nh_.getParam("/zero_state", zero_state_name_);

  state_mobile_manipulation_ = IDLE;

  while (ros::ok())
  {
    ROS_INFO_STREAM("Mobile Manipulator State :" << state_mobile_manipulation_);
    stateMachineMobileManipulation();
    rate.sleep();
    ros::spinOnce();
  }
}

// This function is used in experimentation (integration testing) to store stats for thesis presentation
void writeToCSV(std::string name, Eigen::MatrixXd matrix)
{
  const static Eigen::IOFormat CSV(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
  std::ofstream file(name.c_str());
  file << matrix.format(CSV);
}

