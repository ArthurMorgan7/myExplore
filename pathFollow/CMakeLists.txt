cmake_minimum_required(VERSION 3.0.2)
project(path_track_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  novatel_gps_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_track_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test2 src/test2.cpp)
target_link_libraries(test2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test3 src/test3.cpp)
target_link_libraries(test3 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test4 src/test4.cpp)
target_link_libraries(test4 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test5 src/test5.cpp)
target_link_libraries(test5 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test11 src/test11.cpp)
target_link_libraries(test11 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test12 src/test12.cpp)
target_link_libraries(test12 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test13 src/test13.cpp)
target_link_libraries(test13 ${catkin_LIBRARIES} ${PCL_LIBRARIES})