cmake_minimum_required(VERSION 3.0.2)
project(intellwheels_move_goal)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS
  src/send_goal_robot1
  src/send_goal_robot1.py
  src/send_goal_robot2
  src/send_goal_robot2.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/intellwheels_move_goal
)
install(FILES
  intellwheels_move_goal/src/main.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/intellwheels_move_goal
)

