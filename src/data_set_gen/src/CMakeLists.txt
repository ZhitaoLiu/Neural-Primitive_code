set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.0.2)
project(data_set_gen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
  message_generation

)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

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
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs pcl_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

add_executable(data_set_gen_node 
  src/data_set_gen_node.cpp
  src/map_gen.cpp
  src/primitive_library.cpp
  src/point_cloud_sensing.cpp
  src/plan_vis.cpp
  src/qp_primitive_planner.cpp
  src/init_conditions.cpp
  src/data_set_storage.cpp
  src/data_set_generator.cpp
)
add_dependencies(data_set_gen_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_set_gen_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  OsqpEigen::OsqpEigen
  ${HDF5_LIBRARIES}
)
