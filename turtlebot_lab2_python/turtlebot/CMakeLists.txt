cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_lab2_python)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  trajectory_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  nav_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Install the python executable script(s)
catkin_install_python(PROGRAMS nodes/turtlebot
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

catkin_install_python(PROGRAMS nodes/extended_kf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)
