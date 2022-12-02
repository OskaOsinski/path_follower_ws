cmake_minimum_required(VERSION 2.8.3)
project(spatialdual_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  AngularVelocity.msg
  BodyAcceleration.msg
  ECEFPosition.msg
  FilterStatus.msg
  GeodeticPosition.msg
  GNSSRecvInfo.msg
  NavSatFix.msg
  RollPitchHeading.msg
  SatFixStdDev.msg
  SystemState.msg
  SystemStatus.msg
  UTMPosition.msg
  VelocityNED.msg

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
