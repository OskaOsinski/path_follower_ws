cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  ld_data.msg
  ld_line.msg
  od_data.msg
  od_object.msg
  segmentation_object.msg
  segmentation_data.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  std_msgs
  geometry_msgs
  message_runtime
)
