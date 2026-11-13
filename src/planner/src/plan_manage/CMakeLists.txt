set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pcl_ros
  roscpp
  roslib
  std_msgs
  visualization_msgs
  traj_utils
  plan_env
  cv_bridge
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Torch REQUIRED PATHS "/root/Tools/libtorch") 


add_message_files(
  FILES
  PrimitiveTraj.msg  
)

generate_messages(  
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs pcl_ros roscpp roslib std_msgs visualization_msgs traj_utils plan_env quadrotor_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

add_executable(planner_node 
  src/planner_node.cpp
  src/replan_fsm.cpp
  src/plan_manager.cpp
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TORCH_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_server
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

