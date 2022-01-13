################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(intellwheels_rl)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS rospy)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  src/algorithms/dqn.py
  src/robot_gazebo_nav/base_robot_dqn.py
  src/robot_gazebo_nav/base_robot_environment.py 
  src/robot_gazebo_nav/respawnGoal.py
  src/robot1/robot1_dqn.py
  src/robot1/robot1_environment.py
  src/robot1/respawnGoal.py
  src/robot1/robot1_qlearn.py
  src/robot1/qlearn.py
  src/robot2/robot2_dqn.py
  src/robot2/robot2_environment.py
  src/tools/sample_scan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch save_model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
