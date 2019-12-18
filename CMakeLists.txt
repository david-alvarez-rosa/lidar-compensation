cmake_minimum_required(VERSION 2.8.3)
project(lidar-compensation)

## Compile as C++11, supported in ROS Kinetic and newer.
add_compile_options(-std=c++11)

## Find catkin macros and libraries.
find_package(catkin REQUIRED COMPONENTS roscpp nodelet pcl_ros)

catkin_package(INCLUDE_DIRS include LIBRARIES LidarCompensation)

add_library(LidarCompensation src/LidarCompensation.cc) 

include_directories(include ${catkin_INCLUDE_DIRS})

target_link_libraries(LidarCompensation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(LidarCompensation ${catkin_EXPORTED_TARGETS})

install(TARGETS LidarCompensation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

install(FILES nodelets.xml LidarCompensation.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
