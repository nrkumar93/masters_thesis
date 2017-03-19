cmake_minimum_required(VERSION 2.8.3)
project(csm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig)
pkg_check_modules(Gsl REQUIRED gsl)
add_definitions(-w)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csm
  CATKIN_DEPENDS
  DEPENDS Gsl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Gsl_INCLUDE_DIRS}
)

## CSM requires C-99 standards
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

## Generate JSON config file
INCLUDE(CheckIncludeFile)
MACRO(MY_CHECK_INCLUDE_FILE file VAR)
	CHECK_INCLUDE_FILE(${file} ${file}-exists)
	IF(${${file}-exists})
		SET(${VAR} 1)
	ELSE()
		SET(${VAR} 0)
	ENDIF(${${file}-exists})
ENDMACRO(MY_CHECK_INCLUDE_FILE file VAR)
MY_CHECK_INCLUDE_FILE(fcntl.h HAVE_FCNTL_H)
MY_CHECK_INCLUDE_FILE(inttypes.h HAVE_INTTYPES_H)
MY_CHECK_INCLUDE_FILE(limits.h HAVE_LIMITS_H)
MY_CHECK_INCLUDE_FILE(memory.h HAVE_MEMORY_H)
MY_CHECK_INCLUDE_FILE(stdarg.h HAVE_STDARG_H)
MY_CHECK_INCLUDE_FILE(stdint.h HAVE_STDINT_H)
MY_CHECK_INCLUDE_FILE(stdlib.h HAVE_STDLIB_H)
MY_CHECK_INCLUDE_FILE(strings.h HAVE_STRINGS_H)
MY_CHECK_INCLUDE_FILE(string.h HAVE_STRING_H)
MY_CHECK_INCLUDE_FILE(syslog.h HAVE_SYSLOG_H)
MY_CHECK_INCLUDE_FILE(sys/param.h HAVE_SYS_PARAM_H)
MY_CHECK_INCLUDE_FILE(sys/types.h HAVE_SYS_TYPES_H)
MY_CHECK_INCLUDE_FILE(sys/stat.h HAVE_SYS_STAT_H)
MY_CHECK_INCLUDE_FILE(unistd.h HAVE_UNISTD_H)
MACRO(MY_CHECK_FUNCTION_EXISTS function VAR)
	CHECK_FUNCTION_EXISTS(${function} ${function}-exists)
	IF(${${function}-exists})
		# MESSAGE(STATUS "Exists ${function} = ${${function}-exists}")
		SET(${VAR} 1)
	ELSE()
		# MESSAGE(STATUS "NOT Exists ${function} = ${${function}-exists}")
		SET(${VAR} 0)
	ENDIF(${${function}-exists})
ENDMACRO(MY_CHECK_FUNCTION_EXISTS function VAR)
INCLUDE(CheckFunctionExists)
MY_CHECK_FUNCTION_EXISTS(open HAVE_OPEN)
MY_CHECK_FUNCTION_EXISTS(strubded HAVE_STRUB)
MY_CHECK_FUNCTION_EXISTS(malloc HAVE_MALLOC)
MY_CHECK_FUNCTION_EXISTS(realloc HAVE_REALLOC)
MY_CHECK_FUNCTION_EXISTS(strerror HAVE_STRERROR)
MY_CHECK_FUNCTION_EXISTS(strncasecmp HAVE_STRNCASECMP)
MY_CHECK_FUNCTION_EXISTS(strndup HAVE_STRNDUP)
MY_CHECK_FUNCTION_EXISTS(vasprintf HAVE_VASPRINTF)
MY_CHECK_FUNCTION_EXISTS(vprintf HAVE_VPRINTF)
MY_CHECK_FUNCTION_EXISTS(vsnprintf HAVE_VSNPRINTF)
MY_CHECK_FUNCTION_EXISTS(vsyslog HAVE_VSYSLOG)
CONFIGURE_FILE(include/lib/json-c/config.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/json-c/config.h)


## Declare a cpp library
add_library(csm
  src/csm/clustering.c
  src/csm/gpm/gpm.c
  src/csm/icp/icp.c
  src/csm/icp/icp_corr_dumb.c
  src/csm/icp/icp_corr_tricks.c
  src/csm/icp/icp_covariance.c
  src/csm/icp/icp_loop.c
  src/csm/icp/icp_outliers.c
  src/csm/icp/icp_debug.c
  src/csm/mbicp/mbicp_interface.c      
  src/csm/mbicp/MbICP.c      
  src/csm/mbicp/calcul.c     
  src/csm/mbicp/percolate.c  
  src/csm/mbicp/sp_matrix.c
  src/csm/hsm/hsm.c
  src/csm/hsm/hsm_interface.c
  src/csm/json_journal.c
  src/csm/laser_data.c
  src/csm/laser_data_carmen.c
  src/csm/laser_data_fisher.c
  src/csm/laser_data_json.c
  src/csm/laser_data_load.c
  src/csm/laser_data_drawing.c
  src/csm/laser_data_bbox.c
  src/csm/logging.c
  src/csm/math_utils.c
  src/csm/math_utils_gsl.c
  src/csm/orientation.c
  src/csm/sm_options.c
  src/csm/utils.c
  src/lib/egsl/egsl.c
  src/lib/egsl/egsl_conversions.c
  src/lib/egsl/egsl_misc.c
  src/lib/egsl/egsl_ops.c
  src/lib/gpc/gpc.c
  src/lib/gpc/gpc_utils.c
  src/lib/json-c/arraylist.c
  src/lib/json-c/debug.c
  src/lib/json-c/JSON_checker.c
  src/lib/json-c/json_more_utils.c
  src/lib/json-c/json_object.c
  src/lib/json-c/json_tokener.c
  src/lib/json-c/json_util.c
  src/lib/json-c/linkhash.c
  src/lib/json-c/printbuf.c
  src/lib/options/options.c
  src/lib/options/options_interface.c
)
## Specify libraries to link a library or executable target against
target_link_libraries(csm
  ${catkin_LIBRARIES}
  ${Gsl_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS csm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/csm/ include/lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_csm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
