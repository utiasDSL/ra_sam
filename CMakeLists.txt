cmake_minimum_required(VERSION 3.0.2)
project(ra_sam)

set(CMAKE_CXX_STANDARD 14)

set(default_build_type "RelWithDebInfo") 
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  measurement_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  tf2 
  tf2_ros
  nav_msgs
  sensor_msgs
  measurement_msgs  
)
include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  ${catkin_INCLUDE_DIRS}
  $(GTSAM_INCLUDE_DIRS)
)

add_executable(ra_sam
  src/utils.cpp
  src/ra_sam.cpp
  src/ra_sam_ex.cpp)

  target_link_libraries(ra_sam
   ${catkin_LIBRARIES}
   gtsam
   gtsam_unstable
   yaml-cpp
)

install(TARGETS ra_sam
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}
)
