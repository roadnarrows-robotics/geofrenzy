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
  occupancy_grid_utils
  swri_transform_util
)

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
	FILES
#   Message1.msg
#   Message2.msg
	gf_entitlement.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
#   geographic_msgs#   geometry_msgs#   
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
#link_directories(/usr/local/lib)
include_directories(include $(catkin_INCLUDE_DIRS))
add_executable(gf_map_server src/gf_map_server.cpp)
add_executable(point_cloud_publisher src/testpc.cpp)
add_executable(gf_color_node  src/gf_color.cpp src/hiddata.cpp)
add_executable(gf_color_publisher src/gf_color_pub.cpp)
add_executable(gf_gps_test src/gf_gps_test.cpp)
add_executable(EntitlementToBlink src/EntitlementToBlink.cpp)
add_executable(MapToLaser src/map_to_laser.cpp)
target_compile_options(gf_color_node PUBLIC -std=c++11 -fpermissive)
target_include_directories(gf_color_node PUBLIC usb/blink1/commandline/)
target_include_directories(gf_map_server PUBLIC fa-resolver-libresolv/include/)
target_include_directories(MapToLaser PUBLIC occupancy_grid_utils/include/)
target_link_libraries(point_cloud_publisher ${catkin_LIBRARIES})
target_link_libraries(gf_map_server  ${catkin_LIBRARIES})
target_link_libraries(gf_map_server jsoncpp)
target_link_libraries(gf_gps_test  ${catkin_LIBRARIES})
target_link_libraries(MapToLaser  ${catkin_LIBRARIES})
message( STATUS ${FALIB})
target_link_libraries(gf_map_server ${FALIB} agent)
target_link_libraries(gf_color_node usb ${catkin_LIBRARIES})
target_link_libraries(gf_color_publisher ${catkin_LIBRARIES})
target_link_libraries(EntitlementToBlink ${catkin_LIBRARIES})

#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
#add_executable(blink1_node src/blink1_node.cpp src/hiddata.cpp)
#target_link_libraries(blink1_node usb ${catkin_LIBRARIES})
#add_dependencies(blink1_node blink1_gencpp)
#add_executable(blink1_example examples/blink1_example.cpp)
#target_link_libraries(blink1_example ${catkin_LIBRARIES})
#add_dependencies(blink1_example blink1_gencpp) 


