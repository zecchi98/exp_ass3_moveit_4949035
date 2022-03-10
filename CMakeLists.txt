cmake_minimum_required(VERSION 3.1.3)
project(exp_ass3_moveit_4949035)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  gazebo_msgs
  message_generation
  sensor_msgs
  OpenCV
)
 add_message_files(
   FILES
   ErlOracle.msg
#   Message2.msg
 )
 
 add_service_files(
   FILES
   Oracle.srv
   comunication.srv
#   Service2.srv
 )
 
  generate_messages(
   DEPENDENCIES
   std_msgs
 )
 find_package (OpenCV REQUIRED)
catkin_package(
DEPENDS OpenCV
)


add_library(time STATIC IMPORTED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS scripts/robot_control.py scripts/ID_handler.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
