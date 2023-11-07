cmake_minimum_required(VERSION 3.0.2)
project(car_master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  minimumsnap
)
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES car_master
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(test1 src/test.cpp src/traj.cpp src/multi_traj.cpp)

target_link_libraries(test1
  ${catkin_LIBRARIES}
)


# add_executable(car_master src/car_master.cpp)

# target_link_libraries(car_master
#   ${catkin_LIBRARIES}
# )
