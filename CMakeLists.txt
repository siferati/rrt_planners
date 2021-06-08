cmake_minimum_required(VERSION 3.10.2)
project(rrt_planners)

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++14 -Wall -Wextra -pedantic)

set(CATKIN_DEPENDS roscpp nav_core visualization_msgs tf2 pluginlib nav_msgs)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS ${CATKIN_DEPENDS}
)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/RRTPlanner.cpp src/RRTStarPlanner.cpp src/dubins.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
