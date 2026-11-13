set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

cmake_minimum_required(VERSION 3.0.2)
project(traj_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/planning_visualization.cpp 
)
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    ${EIGEN3_LIBRARIES}
)  

