cmake_minimum_required(VERSION 2.8.3)
project(can_socketcan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package( Boost REQUIRED COMPONENTS
  thread
)

add_message_files( DIRECTORY msg FILES  
  can.msg
)

generate_messages( DEPENDENCIES std_msgs )

catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( can_socketcan_node src/can_socketcan_node.cpp src/SocketCan.cpp )

target_link_libraries(can_socketcan_node
  ${catkin_LIBRARIES}
)

add_dependencies( can_socketcan_node ${PROJECT_NAME} msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
