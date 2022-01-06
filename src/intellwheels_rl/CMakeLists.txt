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
  src/robot_gazebo_nav/base_robot_dqn_stage_1.py
  src/robot_gazebo_nav/base_robot_environment_stage_1.py 
  src/robot_gazebo_nav/respawnGoal.py
  src/robot1/robot1_dqn_stage_1.py
  src/robot1/robot1_environment_stage_1.py
  src/robot1/respawnGoal.py
  src/robot2/robot2_dqn_stage_1.py
  src/robot2/robot2_environment_stage_1.py
  src/robot2/respawnGoal.py
  src/multi/multi_dqn_stage_1.py
  src/multi/multi_environment_stage_1.py
  src/multi/respawnGoal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch save_model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
