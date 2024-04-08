cmake_minimum_required(VERSION 2.8.3)
project(terrain_analysis)

# set(CMAKE_BUILD_TYPE Release)
# set(BUILD_STATIC_LIBS ON)
# set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)



add_executable(terrainAnalysis src/terrainAnalysis.cpp)

target_link_libraries(terrainAnalysis ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

