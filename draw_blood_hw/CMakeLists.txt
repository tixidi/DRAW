cmake_minimum_required(VERSION 2.8.3)
project(draw_blood_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  draw_blood_msgs
  message_runtime 
  serial
  message_generation
)

add_message_files(
  FILES
  Puncture.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)



catkin_package(
  LIBRARIES draw_blood_hw  
  CATKIN_DEPENDS   message_runtime serial
  DEPENDS system_lib

#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/draw_blood_hw/
  /usr/include/alsa

)



add_executable(serial_example_node src/serial_example.cpp src/crc.cpp)
add_dependencies(serial_example_node ${catkin_EXPORTED_TARGETS} ${draw_blood_hw_EXPORTED_TARGETS})
target_link_libraries(serial_example_node ${catkin_LIBRARIES})

add_executable(talker_node src/talker.cpp)
add_dependencies(talker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(talker_node ${catkin_LIBRARIES})
