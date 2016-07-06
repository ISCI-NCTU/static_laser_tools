cmake_minimum_required(VERSION 2.8.3)
project(static_laser_tools)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  rospy
  sensor_msgs
  tf
  std_srvs
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(static_laser_tools
  src/static_laser_tools.cpp
)
target_link_libraries(static_laser_tools
  ${catkin_LIBRARIES}
)

add_executable(closest_point 
    src/closest_point.cpp
    )
target_link_libraries(closest_point
  ${catkin_LIBRARIES}
  static_laser_tools
)




install(TARGETS static_laser_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

