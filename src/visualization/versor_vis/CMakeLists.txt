cmake_minimum_required(VERSION 2.8.3)
project(versor_vis)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rospy
  std_msgs
  message_generation
  visualization_msgs
  driveworks_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs
   visualization_msgs
   driveworks_msgs
   geometry_msgs  # Or other packages containing msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kml_parser_publisher
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
