cmake_minimum_required(VERSION 3.0.2)
project(minimumsnap)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  message_generation
)


find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})



add_message_files(
  FILES
  PolynomialTrajectory.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)
include_directories(
    "./include"
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS} 
)

add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
)

