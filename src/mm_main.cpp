#include "../include/mobile_manip/mobile_manipulation.h"

// Main Function
int main(int argc, char** argv)
{
  ROS_INFO("main");
  ros::init(argc, argv, "MobileManipulation");
  ros::AsyncSpinner spinner(1);
  // Without the spinner the move_group node is unable to fetch states
  // The input to the node is the EE pose, which is read from a topic.

  spinner.start();
  MobileManipulation MobileManipulator;

  MobileManipulator.spin();
}
