cmake_minimum_required(VERSION 2.8.3)
project(control_fsm)

add_compile_options(-std=c++11 -Wall)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      geometry_msgs
      mavros_msgs
      actionlib_msgs
      actionlib
      ascend_msgs
      tf2
      roslaunch
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS message_runtime ascend_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
      ${catkin_INCLUDE_DIRS}
)

file(GLOB control_fsm_SRC "src/fsm/*.cpp")
file(GLOB control_tools_SRC "src/tools/*.cpp")



## Declare a C++ library
#State machine
add_library(control_fsm_lib
        ${control_fsm_SRC}
)
#Control tools not directly related to the FSM
add_library(control_tools_lib
        ${control_tools_SRC}
)

target_link_libraries(control_fsm_lib
        ${catkin_LIBRARIES}
        control_tools_lib
)

## Declare a C++ executable

add_executable(control_fsm_main src/nodes/control_fsm_main)
target_compile_features(control_fsm_main PRIVATE cxx_range_for)

add_executable(land_detector_node src/nodes/land_detector_node)
target_compile_features(land_detector_node PRIVATE cxx_range_for)

## Specify libraries to link a library or executable target against

target_link_libraries(control_fsm_main control_fsm_lib)

target_link_libraries(land_detector_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(control_fsm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(land_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_fsm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_tools_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(utest test/utest.cpp)
target_link_libraries(utest control_fsm_lib)
roslaunch_add_file_check(launch)
