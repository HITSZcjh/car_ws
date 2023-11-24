cmake_minimum_required(VERSION 3.0.2)
project(multi_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_generation
  serial
  dynamic_reconfigure
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
generate_dynamic_reconfigure_options(
   config/car_param.cfg
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
# add_dependencies(multi_traj_node ${PROJECT_NAME}_gencfg)

target_link_libraries(multi_traj_node
  ${catkin_LIBRARIES}
)

add_executable(car_node src/car_node.cpp src/car_controller.cpp)
add_dependencies(car_node ${PROJECT_NAME}_gencfg)

target_link_libraries(car_node
  ${catkin_LIBRARIES}
)

add_executable(sim2real_msg_node src/sim2real_msg.cpp)
target_link_libraries(sim2real_msg_node ${catkin_LIBRARIES})

add_executable(diff_mpc_node src/diff_mpc_node.cpp src/diff_mpc_controller.cpp)
target_include_directories(diff_mpc_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
  ${PROJECT_SOURCE_DIR}/scripts/c_generated_code/diff_car_controller
)

target_link_directories(diff_mpc_node PUBLIC
${catkin_LIB_DIRS}
$ENV{ACADOS_SOURCE_DIR}/lib
${PROJECT_SOURCE_DIR}/scripts/c_generated_code/diff_car_controller)

target_link_libraries(diff_mpc_node acados_ocp_solver_diff_car_controller acados hpipm blasfeo ${catkin_LIBRARIES})


# add_executable(car_master src/car_master.cpp)

# target_link_libraries(car_master
#   ${catkin_LIBRARIES}
# )
