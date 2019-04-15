cmake_minimum_required(VERSION 2.8.3)
project(jacob_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #moveit_core
  #moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  #pluginlib
  #cmake_modules
  #geometric_shapes
)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning_group_LIBRARIES
  CATKIN_DEPENDS
    #moveit_core
    #moveit_ros_planning_interface
    #interactive_markers
 # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#----------------------------------- add planning_group LIBRARIES -------------------------------------------------------------#

add_library(planning_group_LIBRARIES
  src/PlanningGroup.cpp
)
target_link_libraries(planning_group_LIBRARIES ${catkin_LIBRARIES})
add_dependencies(planning_group_LIBRARIES ${catkin_EXPORTED_TARGETS})

#----------------------------------- add object_creator LIBRARIES -------------------------------------------------------------#
add_library(object_creator_LIBRARIES
  src/ObjectCreator.cpp
)
target_link_libraries(object_creator_LIBRARIES ${catkin_LIBRARIES})
add_dependencies(object_creator_LIBRARIES ${catkin_LIBRARIES})

#----------------------------------- add robot_pose executable -------------------------------------------------------------#

add_executable(robot_pose src/node/robot_pose.cpp)
target_link_libraries(robot_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS robot_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#----------------------------------- add moveto executable -------------------------------------------------------------#

add_executable(moveto src/node/moveto.cpp)
add_dependencies(moveto ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveto planning_group_LIBRARIES object_creator_LIBRARIES ${catkin_LIBRARIES})
#install(TARGETS moveto DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#----------------------------------- add mark_detection executable -------------------------------------------------------------#

add_executable(mark_detection src/node/mark_detection.cpp)
target_link_libraries(mark_detection planning_group_LIBRARIES object_creator_LIBRARIES ${catkin_LIBRARIES})


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
