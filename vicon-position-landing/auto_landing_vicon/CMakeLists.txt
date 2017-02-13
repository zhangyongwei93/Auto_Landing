cmake_minimum_required(VERSION 2.8.3)
project(vicon_position_auto_landing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_extras
  geometry_msgs
  tf2_ros
  eigen_conversions
  angles
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

include(EnableCXX11)



catkin_package(
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(vicon_position_auto_landing src/vicon_position_auto_land.cpp)

target_link_libraries(vicon_position_auto_landing
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

