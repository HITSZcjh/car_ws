cmake_minimum_required(VERSION 3.0.2)
project(multi_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_generation
  serial
)
find_package(Eigen3 REQUIRED)
add_message_files(
  FILES
  ContorlRef.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(
#  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_traj_node src/multi_traj_node.cpp src/traj.cpp src/multi_traj.cpp src/serial_send.cpp)

target_link_libraries(multi_traj_node
  ${catkin_LIBRARIES}
)

add_executable(car_node src/car_node.cpp src/car_controller.cpp)

target_link_libraries(car_node
  ${catkin_LIBRARIES}
)

# add_executable(car_master src/car_master.cpp)

# target_link_libraries(car_master
#   ${catkin_LIBRARIES}
# )
