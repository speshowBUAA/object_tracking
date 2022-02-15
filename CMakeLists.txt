cmake_minimum_required(VERSION 2.8.3)
project(object_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std = c++ 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_compile_options(-std=c++11 -O3 -g -DNDEBUG -fopenmp -pthread)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -DDEBUG -ggdb")
# add_compile_options(-std=c++11 -pthread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf2_ros
  eigen_conversions
  bayes_tracking
  autoware_msgs
)

find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    tf2_ros
    eigen_conversions
    bayes_tracking
    autoware_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/object_tracking_node.cpp
  src/object_tracking.cpp
  src/object_tracker/object_tracker.cpp
)

add_executable(object_tracking
  ${SOURCE_FILES}
)

add_dependencies(object_tracking
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_tracking
  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} glog
)

add_executable(object_tracking_test 
    test/object_tracking_test.cpp
    src/object_tracker/object_tracker.cpp
)

target_link_libraries(object_tracking_test ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} glog)

install(
  TARGETS object_tracking_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)