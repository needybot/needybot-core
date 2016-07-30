cmake_minimum_required(VERSION 2.8.3)
project(needybot)

include(ExternalProject)

set(PKG_PREFIX ${CMAKE_SOURCE_DIR})
ExternalProject_add(
  pi_trees
  PREFIX ${PKG_PREFIX}
  DOWNLOAD_DIR ${PKG_PREFIX}/pi_trees
  TMP_DIR ${PKG_PREFIX}/.tmp/pi_trees
  STAMP_DIR ${CMAKE_SOURCE_DIR}/.stamps/pi_trees
  GIT_REPOSITORY https://github.com/pirobot/pi_trees.git 
  GIT_TAG indigo-devel
  UPDATE_COMMAND ""
  SOURCE_DIR ${PKG_PREFIX}/pi_trees
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
)

ExternalProject_add(
  needybot_msgs 
  PREFIX ${PKG_PREFIX}
  DOWNLOAD_DIR ${PKG_PREFIX}/needybot-msgs
  TMP_DIR ${PKG_PREFIX}/.tmp/needybot-msgs
  STAMP_DIR ${CMAKE_SOURCE_DIR}/.stamps/needybot-msgs
  GIT_REPOSITORY https://github.com/needybot/needybot-msgs.git 
  GIT_TAG master 
  UPDATE_COMMAND ""
  SOURCE_DIR ${PKG_PREFIX}/needybot-msgs
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
)

ExternalProject_add(
  needybot_srvs
  PREFIX ${PKG_PREFIX}
  DOWNLOAD_DIR ${PKG_PREFIX}/needybot-srvs
  TMP_DIR ${PKG_PREFIX}/.tmp/needybot-srvs
  STAMP_DIR ${CMAKE_SOURCE_DIR}/.stamps/needybot-srvs
  GIT_REPOSITORY https://github.com/needybot/needybot-srvs.git 
  GIT_TAG master 
  UPDATE_COMMAND ""
  SOURCE_DIR ${PKG_PREFIX}/needybot-srvs
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
)



# Run install.sh
execute_process(
  COMMAND
  ${CMAKE_CURRENT_SOURCE_DIR}/install.sh ${CMAKE_PREFIX_PATH} ${CMAKE_PREFIX_PATH}
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  kobuki_msgs
  needybot_msgs 
  needybot_srvs 
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  dynamic_reconfigure
)
find_package(Boost REQUIRED)
find_package(PCL)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


catkin_python_setup()
generate_dynamic_reconfigure_options(
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    needybot_msgs 
    needybot_srvs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    message_runtime
    dynamic_reconfigure
  DEPENDS Boost PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

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
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(FILES plugins/recoveries.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY sim
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_needybot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/needybot.test)
endif()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
