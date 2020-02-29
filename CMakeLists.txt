cmake_minimum_required(VERSION 2.8.3)
project(perception)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp)
find_package(PCL REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      roscpp
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


## Declare a cpp library
add_library(${PROJECT_NAME}
    src/apps/pcReg/align_icp.cpp
    src/apps/pcReg/align_prerejective.cpp
    src/apps/pcReg/load_parameters.cpp
    src/apps/pcReg/load_pc.cpp
    src/apps/pcReg/run_register.cpp
    src/mUtils/noise_reduce.cpp
    src/mUtils/mFunctions.cpp
)

## executable
add_executable(run_apps src/run_apps.cpp)

target_link_libraries(run_apps 
${PROJECT_NAME} 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
)

## executable
add_executable(pcl_converter src/pcl_converter.cpp)

target_link_libraries(pcl_converter 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
)

## executable
add_executable(prerejective_align src/prerejective_align.cpp)

target_link_libraries(prerejective_align 
${PROJECT_NAME} 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
)
