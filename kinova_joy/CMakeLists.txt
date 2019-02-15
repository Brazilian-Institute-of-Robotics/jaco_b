cmake_minimum_required(VERSION 2.8.3)
project(kinova_joy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  message_generation
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinova_joy
#  CATKIN_DEPENDS gazebo joy roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_interface src/joy_interface.cpp)
target_link_libraries(joy_interface ${catkin_LIBRARIES})
add_dependencies(joy_interface ${catkin_EXPORTED_TARGETS} )

add_executable(home_position src/home_position.cpp)
target_link_libraries(home_position ${catkin_LIBRARIES})
add_dependencies(home_position ${catkin_EXPORTED_TARGETS} )

add_executable(joy_hposition src/joy_hposition.cpp)
target_link_libraries(joy_hposition ${catkin_LIBRARIES})
add_dependencies(joy_hposition ${catkin_EXPORTED_TARGETS} )
