cmake_minimum_required(VERSION 3.0.2)
project(slam_si)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  fsd_common_msgs
  roscpp
  sgtdv_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/slam_si.cpp src/slam_si_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
