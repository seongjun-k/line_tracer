cmake_minimum_required(VERSION 3.0.2)
project(line_tracer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/line_tracer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
