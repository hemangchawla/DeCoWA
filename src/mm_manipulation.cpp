#include "../include/mobile_manip/mm_manipulation.h"

namespace mm_manipulation
{
bool executeMotion(moveit_msgs::DisplayTrajectory& trajectory_msg, bool simulation_publish,
                   unsigned int publishing_rate, std::vector<int> joint_handles)
{
  ros::NodeHandle nh;
  ros::Publisher Publisher_arm_motion_real = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  // Arm motion vrep - advertizing the publication of the corresponding joint positions to reach for executing the
  // trajectory
  ros::Publisher Publisher_arm_motion_sim = nh.advertise<vrep_common::JointSetStateData>("/vrep/joint_path", 1);

  // Return only after execution is complete
  unsigned int point_count = trajectory_msg.trajectory[0].joint_trajectory.points.size();
  ROS_INFO_STREAM("Publishing " << point_count << " points in trajectory");
  if (point_count == 0)
  {
    ROS_WARN("No points in trajectory!");
    return false;
  }

  unsigned int num_joints = trajectory_msg.trajectory[0].joint_trajectory.points[0].positions.size();
  sensor_msgs::JointState joint_states;
  vrep_common::JointSetStateData joint_positions;

  Eigen::MatrixXd motion_matrix;

  for (int i = 0; i < point_count; i++)
  {
    for (int j = 0; j < num_joints; j++)
    {
      motion_matrix.conservativeResize(i + 1, num_joints);
      motion_matrix(i, j) = (trajectory_msg.trajectory[0].joint_trajectory.points[i].positions[j]);
    }
  }

  for (int k = 0; k < point_count; k++)  // Loop over the way points
  {
    for (int l = 0; l < num_joints; l++)
    {
      // For joint states update
      joint_states.name.push_back(trajectory_msg.trajectory[0].joint_trajectory.joint_names[l]);
      joint_states.position.push_back(motion_matrix(k, l));

      if (simulation_publish)
      {
        // Set the joint handles for which the joint positions will be set
        joint_positions.handles.data.push_back(joint_handles[l]);
        // This tells that the positions will be published and not velocity or acceleration of the joints
        joint_positions.setModes.data.push_back(0);
        // Set the required joint positions corresponding to this waypoint
        joint_positions.values.data.push_back(motion_matrix(k, l));
      }
    }

    // Publish the joint positions for this way point
    Publisher_arm_motion_real.publish(joint_states);
    // Clear after publishing
    joint_states.name.clear();
    joint_states.position.clear();

    if (simulation_publish)
    {
      Publisher_arm_motion_sim.publish(joint_positions);
      // Clear the joint_positions variable so the next way point can be looped over
      joint_positions.values.data.clear();
      joint_positions.handles.data.clear();
      joint_positions.setModes.data.clear();
    }
    ros::Rate(publishing_rate).sleep();
    ros::spinOnce();
  }
  return true;
}

bool unfoldArm(std::vector<double>& folded_positions, std::vector<double>& unfolded_positions,
               std::vector<std::string>& joint_names, bool simulation_publish, std::vector<int> joint_handles)
{
  /*
   * Options:
   * 1. Uniform motion between the current config to target config (Faster but may not be generalizable)
   * 2. Motion Planning using STOMP (Takes longer, More general because avoids obstacles)
   * SELECTED: 1, because is simpler
   */

  // Compute the linear interpolation between states.
  // Send back signal only after unfolding is complete (based on signal from execution)

  unsigned int discretization = 100;

  unsigned int num_joints = unfolded_positions.size();
  moveit_msgs::DisplayTrajectory display_trajectory_msg;
  moveit_msgs::RobotTrajectory robot_trajectory;

  // dq is used as mathematical notation determining the difference between the configurations
  std::vector<double> dq;

  for (int jnt = 0; jnt < num_joints; ++jnt)
  {
    dq.push_back((unfolded_positions[jnt] - folded_positions[jnt]) / discretization);
  }

  // discretize the motion
  for (int i = 0; i < discretization + 1; i++)
  {
    trajectory_msgs::JointTrajectoryPoint pt;

    for (int j = 0; j < num_joints; ++j)
    {
      pt.positions.push_back(folded_positions[j] + i * dq[j]);
    }
    robot_trajectory.joint_trajectory.points.push_back(pt);
  }

  // Visualize trajectory msg
  for (int k = 0; k < joint_names.size(); ++k)
  {
    robot_trajectory.joint_trajectory.joint_names.push_back(joint_names[k]);
  }
  display_trajectory_msg.trajectory.push_back(robot_trajectory);

  // Execute unfolding motion
  bool executed = executeMotion(display_trajectory_msg, simulation_publish, SPIN_RATE, joint_handles);

  if (executed)
  {
    ROS_INFO("Arm_unfolding completed");
  }
  return executed;
}

bool computeMotionPlan(moveit::planning_interface::MoveGroupInterfacePtr& group_, robot_state::RobotState& start_state,
                       geometry_msgs::Pose& target)
{
  std::string planner_id;

  if (ros::param::get("/arm_motion_planner", planner_id))
  {
    group_->setPlannerId(planner_id);
  }
  else
  {
    ROS_WARN("Planner not specified. Will use default");
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_->setStartState(start_state);
  group_->setPoseTarget(target);
  // @todo: using the principle of single point of control use a rosparam to get the goal tolerance to be used
  group_->setGoalTolerance(0.001);
  bool success = group_->plan(my_plan);
  return success;
}
}
