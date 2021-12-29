cmake_minimum_required(VERSION 3.0.2)
project(intellwheels_echo)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python(PROGRAMS
  listener
  listener.py
  talker
  talker.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/intellwheels_echo
)
install(FILES
  intellwheels_echo/talker_listener.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/intellwheels_echo
)

