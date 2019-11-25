cmake_minimum_required(VERSION 2.8.3)
project(repub_lcm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)


set( hearders
  src/rostolcm_gga_t.hpp
  src/rostolcm_imu_t.hpp
  src/rostolcm_rmc_t.hpp
  src/rostolcm_pos_t.hpp
)

set( sources
  src/repub_lcm_node.cpp
)


find_package(lcm REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${sources} ${headers})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   lcm
)


