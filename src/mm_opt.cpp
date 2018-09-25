#include "../include/mobile_manip/mm_opt.h"

namespace mm_opt
{
namespace details
{
void init(tf::Vector3 test_arm_tf, planning_scene::PlanningScenePtr& scene, Eigen::Affine3d& link0_to_base_link,
          std::string& base_group_name, geometry_msgs::Pose2D& current_base_pose)
{
  // set number of iteration to zero since optimization has not begun
  iter = 0;
  arm_location = test_arm_tf;
  scene_ptr_ = scene;
  link0_to_base_tf = link0_to_base_link;
  current_pose = current_base_pose;
  const robot_model::RobotModelConstPtr kinematic_model = scene->getRobotModel();
  base_joint_model_group = kinematic_model->getJointModelGroup(base_group_name);
  collision_request.group_name = base_group_name;
  collision_request.distance = false;
  collision_request.cost = false;
}

void generateTestStateConfiguration(double angle, std::vector<double>& whole_joint_config)
{
  // Make a frame
  tf::Transform test_arm_tf;
  tf::Quaternion arm_quat;
  // So angle is wrt the link0 angle
  arm_quat.setEuler(0, 0, angle);
  test_arm_tf.setOrigin(arm_location);
  test_arm_tf.setRotation(arm_quat);

  Eigen::Affine3d test_arm_frame;
  tf::transformTFToEigen(test_arm_tf, test_arm_frame);

  Eigen::Affine3d test_base_frame = test_arm_frame * link0_to_base_tf;

  tf::Transform test_base_location;
  tf::transformEigenToTF(test_base_frame, test_base_location);

  // Move the base pose to the correponding location (since arm cannot be move alone)
  whole_joint_config[0] = test_base_location.getOrigin().getX();
  whole_joint_config[1] = test_base_location.getOrigin().getY();
  whole_joint_config[2] = tf::getYaw(test_base_location.getRotation());
  // Rest of them remain the same as the test state, i.e. zero.
}

/// The variable naming is based on the convention of NLOPT, as it is optimizaing for x using the gradient grad and
/// additional data for computation
double objectiveFunction(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Whenever the optimization algo accesses the objectiveFunction value ie the cost increase the iter by 1
  ++iter;

  // Update state at test config
  std::vector<double> base_joint_group_positions = { 0, 0, 0 };
  generateTestStateConfiguration(x[0], base_joint_group_positions);
  robot_state::RobotState state = scene_ptr_->getCurrentStateNonConst();
  state.setJointGroupPositions(base_joint_model_group, base_joint_group_positions);
  collision_result.clear();
  state.updateCollisionBodyTransforms();

  // Check for collision of base at test config (Onl base is checked as the arm was checked while determining the arm
  // location)
  scene_ptr_->getCollisionWorld()->checkRobotCollision(collision_request, collision_result,
                                                       *(scene_ptr_->getCollisionRobot()), state,
                                                       scene_ptr_->getAllowedCollisionMatrix());

  // If no collision, the output is the distance of base from the current pose
  if (!collision_result.collision)
  {
    return pow(base_joint_group_positions[0] - current_pose.x, 2) +
           pow(base_joint_group_positions[1] - current_pose.y, 2);
  }
  else
  {
    // If in collision the cost is huge
    return HUGE_VALF;
  }
}

double wrapAngle(double angle)
{
  angle = fmod(angle, 2 * M_PI);

  if (angle < 0)
  {
    angle += 2 * M_PI;
  }
  return angle;
}
}

double solve(tf::Vector3 test_arm_tf, planning_scene::PlanningScenePtr& scene, Eigen::Affine3d& link0_to_base_link,
             std::string& group_name, geometry_msgs::Pose2D& current_base_pose)
{
  tf::Transform arm_to_base;
  tf::transformEigenToTF(link0_to_base_link, arm_to_base);

  // Initialize the optimizer
  details::init(test_arm_tf, scene, link0_to_base_link, group_name, current_base_pose);

  // NL Neldermead method is used for optimization
  nlopt::opt optim(nlopt::LN_NELDERMEAD, 1);

  // lb are the lower bound for the single (1) variable ie yaw
  std::vector<double> lb(1);
  // setting lower bound to zero
  lb[0] = 0;
  optim.set_lower_bounds(lb);

  // ub are the upper bound for the single (1) variable ie yaw
  std::vector<double> ub(1);
  // setting upper bound to 2*pi
  ub[0] = 2 * M_PI;
  optim.set_upper_bounds(ub);

  // We want to minimize the objective function
  optim.set_min_objective(details::objectiveFunction, NULL);

  // Params of tolerance and max number of evaluations
  optim.set_xtol_rel(1e-3);
  optim.set_maxeval(100);

  std::vector<double> theta(1);
  theta[0] = details::wrapAngle(details::current_pose.theta + tf::getYaw(arm_to_base.getRotation()));

  // minf is the value of function at min cost
  double minf;
  nlopt::result res = optim.optimize(theta, minf);

  // If the minimum could be found
  if (res > 0)
  {
    return theta[0];
  }
  else
  {
    return NAN;
  }
}
}
