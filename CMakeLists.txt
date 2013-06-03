cmake_minimum_required(VERSION 2.8.3)
project(ardu_pilot)

set(catkin_DIR /opt/ros/groovy/share/catkin/cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp serial std_msgs sensor_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    command.msg
    Mavlink.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
  LIBRARIES ardu_pilot
  CATKIN_DEPENDS roscpp serial std_msgs sensor_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
#link_directories()

#set(mavlink_DIR include/mavlink/v1.0/ardupilotmega)

set(ardu_pilot_SRCS 
    src/ardu_pilot.cpp
    include/ardu_pilot/ardu_pilot.h
#    ${mavlink_DIR}/mavlink.h
#    ${mavlink_DIR}/version.h
#    ${mavlink_DIR}/ardupilotmega.h

)


## Declare a cpp library
 add_library(${PROJECT_NAME}  ${ardu_pilot_SRCS})


## Declare a cpp executable
# add_executable(ardu_pilot_node src/ardu_pilot_node.cpp)
# add_executable(ardu_pilot_listener src/ardu_pilot_listener.cpp)

 add_executable(ardu_pilot_node src/ardu_pilot_node.cpp src/ardu_pilot.cpp)
 add_executable(ardu_pilot_listener src/ardu_pilot_listener.cpp src/ardu_pilot.cpp)

## Add dependencies to the executable
 add_dependencies(ardu_pilot_node ${PROJECT_NAME})
 add_dependencies(ardu_pilot_listener ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
 target_link_libraries(ardu_pilot_node ${catkin_LIBRARIES})
 target_link_libraries(ardu_pilot_listener ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ardu_pilot ardu_pilot_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ardu_pilot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
