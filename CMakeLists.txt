cmake_minimum_required(VERSION 2.8.3)
project(mobile_manip)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11)
add_compile_options(-fext-numeric-literals)


find_package(catkin REQUIRED COMPONENTS
   actionlib
   actionlib_msgs
   cv_bridge
   kdl_conversions
   map_creator
   moveit_core
   moveit_msgs
   moveit_ros_move_group
   moveit_ros_planning
   moveit_ros_planning_interface
   moveit_ros_visualization
   moveit_ros_manipulation
   moveit_visual_tools
   nlopt
   pluginlib cmake_modules
   roscpp
   rospy
   sensor_msgs
   std_msgs
   tf
   trac_ik_lib
   trac_ik_kinematics_plugin
   geometry_msgs
   vrep_common
   vrep_plugin
   vrep_ros_interface
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})
include_directories(${NLOPT_INCLUDE_DIRS})
link_directories(${NLOPT_LIBRARY_DIRS})

catkin_package()

include_directories(
	/include/
	${PROJECT_NAME}/include/
  	${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GSTREAMER_INCLUDE_DIRS}
)


add_library(mm_opt  src/mm_opt.cpp)
target_link_libraries(mm_opt ${catkin_LIBRARIES} -lnlopt_cxx)

add_library(mm_robot_placement  src/mm_robot_placement.cpp)
target_link_libraries(mm_robot_placement mm_opt ${catkin_LIBRARIES})

add_library(mm_manipulation  src/mm_manipulation.cpp)
target_link_libraries(mm_manipulation ${catkin_LIBRARIES})

add_library(mm_locomotion  src/mm_locomotion.cpp)
target_link_libraries(mm_locomotion ${catkin_LIBRARIES})

add_library(mobile_manipulation src/mobile_manipulation.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mobile_manipulation
  ${catkin_LIBRARIES}
  mm_robot_placement
  mm_manipulation
  mm_locomotion
)

add_executable(mm_main src/mm_main.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mm_main
  ${catkin_LIBRARIES}
  mm_robot_placement
  mm_manipulation
  mm_locomotion
  mobile_manipulation
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
