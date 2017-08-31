cmake_minimum_required(VERSION 2.8.3)
project(geofrenzy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geodesy
  geographic_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  swri_transform_util
  cv_bridge
)

if(NOT ${EIGEN3_FOUND})
  message(STATUS "Eigen3 not found - hardcode directory")
  set(Eigen3_DIR /usr/lib/cmake/eigen3)
  find_package(Eigen3 REQUIRED)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
	FILES
    GeoPolygon.msg
    GfDistFeature.msg
    GfDistFeatureCollection.msg
    GfDwellBoolset.msg
    GfDwellColor.msg
    GfDwellJson.msg
    GfDwellProfile.msg
    GfDwellThreshold.msg
    GfEntBoolset.msg
    GfEntColor.msg
    GfEntHeader.msg
    GfEntJson.msg
    GfEntProfile.msg
    GfEntThreshold.msg
    GfEntTVP.msg
    GfGeoFeature.msg
    GfGeoFeatureCollection.msg
    GfJsonFeatureCollection.msg
    Polygon64.msg
    StringStamped.msg
    entitlement.msg   # RDK deprecated
    gf_class.msg      # RDK deprecated
)

## Generate services in the 'srv' folder
 add_service_files(
  DIRECTORY srv
	FILES
    GetEntitlement.srv
    GetEntitlementList.srv
    entitlement_service.srv         # RDK deprecated
    entitlement_list_service.srv    # RDK deprecated
    class_service.srv               # RDK deprecated
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geographic_msgs
    geometry_msgs  
	  std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES geofrenzy
#  CATKIN_DEPENDS geodesy geographic_msgs geometry_msgs roscpp rospy std_msgs tf
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(geofrenzy
#   src/${PROJECT_NAME}/geofrenzy.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(geofrenzy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(geofrenzy_node src/geofrenzy_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(geofrenzy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(geofrenzy_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS geofrenzy geofrenzy_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_geofrenzy.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

find_library(FALIB faresolv HINTS /usr/local/lib)

if(NOT FALIB)
	message(FATAL_ERROR "fa library not found")
endif()
#message( STATUS ${FALIB})
include_directories(
  src
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# gprof profiler compile and link options
# Uncomment to use
#set(GPROF_OPTS PUBLIC -pg -fprofile-arcs)

# Geofrenzy map server node
add_executable(gf_map_server
  src/gf_ros.cpp
  src/gf_map_server.cpp)
target_link_libraries(gf_map_server  ${catkin_LIBRARIES})

# geofrenzy portal server
add_executable(gf_server
  src/gf_ros.cpp
  src/gf_server.cpp)
target_link_libraries(gf_server ${catkin_LIBRARIES})
target_link_libraries(gf_server jsoncpp)
target_link_libraries(gf_server ${FALIB} agent)

# geofrenzy RGBD virtual sensor
add_executable(gf_vcloud
  src/gf_math.cpp
  src/gf_ros.cpp
  src/gf_vcloud.cpp)
target_link_libraries(gf_vcloud
  ${GPROF_OPTS}
  ${catkin_LIBRARIES})
#target_compile_options(gf_vcloud ${GPROF_OPTS})

# geofrenzy sensor relay
add_executable(gf_sensor_relay
  src/gf_ros.cpp
  src/gf_sentinel.cpp
  src/gf_sensor_relay.cpp)
target_link_libraries(gf_sensor_relay ${catkin_LIBRARIES})

install(
  PROGRAMS 
    scripts/latlongmeters.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
