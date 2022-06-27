cmake_minimum_required(VERSION 2.8.3)
project(haptic_waypoint_replanner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
add_compile_options(-O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(haptic_waypoint_replanner_node src/haptic_waypoint_replanner_node.cpp)
target_link_libraries(haptic_waypoint_replanner_node ${catkin_LIBRARIES})
add_dependencies(haptic_waypoint_replanner_node ${PROJECT_NAME}_gencpp )

foreach(launch)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

