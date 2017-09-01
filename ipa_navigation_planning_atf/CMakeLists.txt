cmake_minimum_required(VERSION 2.8.3)
project(ipa_navigation_planning_atf)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  atf_core
  geometry_msgs
  #ipa_navigation_planning
  roscpp
  roslaunch
  rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   CATKIN_DEPENDS
     roscpp
     rospy
   INCLUDE_DIRS include
   LIBRARIES ipa_navigation_planning_atf
)

###########
## Build ##
###########

add_executable(robot_route_recorder
  src/robot_route_recorder.cpp
)

target_link_libraries(robot_route_recorder
  ${catkin_LIBRARIES}
)

add_dependencies(robot_route_recorder
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(PROGRAMS scripts/application.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
roslaunch_add_file_check(launch)
atf_test()
