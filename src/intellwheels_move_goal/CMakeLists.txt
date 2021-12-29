cmake_minimum_required(VERSION 3.0.2)
project(intellwheels_move_goal)

find_package(catkin REQUIRED)


catkin_package()

catkin_python_setup()


catkin_install_python(PROGRAMS 
    src/talker.py
    src/listener.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/main.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

