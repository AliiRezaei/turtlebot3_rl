cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_rl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	std_msgs
	nav_msgs
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
# DEPENDS other non-ROS libs
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/turtlebot3_rl_node.cpp)


add_executable(learning_empty_world src/learning_empty_world.cpp)
target_link_libraries(learning_empty_world ${catkin_LIBRARIES})

add_executable(main_empty_world src/main_empty_world.cpp)
target_link_libraries(main_empty_world ${catkin_LIBRARIES})

add_executable(learning_world src/learning_world.cpp)
target_link_libraries(learning_world ${catkin_LIBRARIES})

add_executable(main_world src/main_world.cpp)
target_link_libraries(main_world ${catkin_LIBRARIES})

add_executable(learning_house src/learning_house.cpp)
target_link_libraries(learning_house ${catkin_LIBRARIES})

add_executable(main_house src/main_house.cpp)
target_link_libraries(main_house ${catkin_LIBRARIES})