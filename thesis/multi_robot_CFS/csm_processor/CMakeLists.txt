cmake_minimum_required(VERSION 2.8.3)
project(csm_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
  csm
  csm_ros
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(GTSAM REQUIRED)
set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csm_processor
  CATKIN_DEPENDS roscpp rospy std_msgs
  sensor_msgs
  tf2
  csm
  csm_ros  
  DEPENDS Boost GTSAM
)

###########
## Build ##
###########

## Use C++11
add_definitions(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

add_library(csm_processor
  include/csm_processor/csm_processor.h
  src/csm_processor.cpp
)
target_link_libraries(csm_processor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GTSAM_LIBRARIES}
)

## Executable for calling csm_processor
add_executable(csm_caller
    src/csm_caller.cpp
)

target_link_libraries(csm_caller
  ${catkin_LIBRARIES}
  csm_processor
)
  
add_dependencies(csm_processor 
  ${csm_processor_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS csm_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/csm_processor/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
