cmake_minimum_required(VERSION 2.8.3)
project(control_fsm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
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
	CATKIN_DEPENDS message_runtime
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

file(GLOB control_fsm_SRC "src/*.cpp")

## Declare a C++ library
add_library(control_fsm_lib
  ${control_fsm_SRC}
)
target_compile_features(control_fsm_lib PRIVATE cxx_range_for)

target_link_libraries(control_fsm_lib
	${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(control_fsm_test src/nodes/control_fsm_test)
target_compile_features(control_fsm_test PRIVATE cxx_range_for)

add_executable(control_fsm_main src/nodes/control_fsm_main)
target_compile_features(control_fsm_main PRIVATE cxx_range_for)

add_executable(control_fsm_gazebo_test src/nodes/control_fsm_gazebo_test)
target_compile_features(control_fsm_gazebo_test PRIVATE cxx_range_for)

add_executable(control_fsm_gazebo_input src/nodes/control_fsm_gazebo_input)
target_compile_features(control_fsm_gazebo_input PRIVATE cxx_range_for)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(control_fsm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control_fsm_test
  	control_fsm_lib
)

target_link_libraries(control_fsm_main
	control_fsm_lib
)

target_link_libraries(control_fsm_gazebo_test
	control_fsm_lib
)

target_link_libraries(control_fsm_gazebo_input
	control_fsm_lib
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(control_fsm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_fsm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_fsm_gazebo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_fsm_gazebo_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_fsm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})