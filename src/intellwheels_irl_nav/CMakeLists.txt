cmake_minimum_required(VERSION 3.0.2)
project(intellwheels_irl_nav)

set(CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf2)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})

catkin_package(CATKIN_DEPENDS ${CATKIN_DEPENDS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_odom_twister src/OdomTwister.cpp)

target_link_libraries(${PROJECT_NAME}_odom_twister ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}_odom_twister PROPERTIES OUTPUT_NAME odom_twister)