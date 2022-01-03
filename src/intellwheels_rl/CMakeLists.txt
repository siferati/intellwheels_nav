cmake_minimum_required(VERSION 3.0.2)
project(intellwheels_rl)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

catkin_install_python()
install()

