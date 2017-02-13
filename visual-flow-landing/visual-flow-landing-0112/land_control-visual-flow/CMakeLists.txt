cmake_minimum_required(VERSION 2.8.3)
project(velocity_marker_land)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(
#	cfg/tracking_pid.CMAKE_CFG_INTDIR
#	)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(velocity_marker_land_node src/velocity_marker_land_node.cpp)

#add_executable(tracking_pid_node src/tracking_pid.cpp)

#add_dependencies(velocity_marker_tracker_node $(velocity_marker_tracker)_gencfg)


## Specify libraries to link a library or executable target against
 target_link_libraries(velocity_marker_land_node
   ${catkin_LIBRARIES}
 )

