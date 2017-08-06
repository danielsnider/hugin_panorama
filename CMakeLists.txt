cmake_minimum_required(VERSION 2.8.3)

project(hugin_panorama)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
  std_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs rospy std_msgs cv_bridge
)

catkin_install_python(
  PROGRAMS nodes/hugin_panorama
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install scripts
install(
  PROGRAMS scripts/stitch_loop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)