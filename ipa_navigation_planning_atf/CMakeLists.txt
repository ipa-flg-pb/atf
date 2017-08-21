cmake_minimum_required(VERSION 2.8.3)
project(ipa_navigation_planning_atf)

find_package(catkin REQUIRED COMPONENTS
  atf_core
  #ipa_navigation_planning
  roslaunch
  rospy
)
catkin_package(
   CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

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
