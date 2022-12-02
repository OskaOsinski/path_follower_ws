cmake_minimum_required(VERSION 2.8.3)
project(niro_demo_launcher )

message("Invoking setup script for package under path: " ${CMAKE_CURRENT_SOURCE_DIR}/script/setup.sh)
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/script/setup.sh
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

message("Invoking setup script for adding desktop shortcut: " ${CMAKE_CURRENT_SOURCE_DIR}/script/createShortcut.sh)
message("Setting up ros workspace to: " ${CMAKE_BINARY_DIR}/../)
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/script/createShortcut.sh ${CMAKE_BINARY_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

find_package(catkin REQUIRED COMPONENTS
  message_generation
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
