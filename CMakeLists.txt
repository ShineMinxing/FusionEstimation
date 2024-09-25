cmake_minimum_required(VERSION 3.10)
project(fusion_estimator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rospy
        urdf
        roslib
        nav_msgs
        std_msgs
        sensor_msgs
        ocs2_quadruped_msgs
        message_generation
        dynamic_reconfigure
        )

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/EstimatorConfig.cfg
)

add_message_files(
  FILES
  FusionEstimatorTest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        urdf
        roslib
        std_msgs
        sensor_msgs
        message_runtime
        dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "CATKIN_DEVEL_PREFIX include: ${CATKIN_DEVEL_PREFIX}/include")

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/FusionEstimation.cpp
        src/Sensors.cpp
        )

#############
add_executable(${PROJECT_NAME}_node
        src/FusionEstimationNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
