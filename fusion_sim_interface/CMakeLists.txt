cmake_minimum_required(VERSION 3.0.2)
project(fusion_sim_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
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

add_executable(${PROJECT_NAME} src/fusion_sim_interface.cpp src/fusion_sim_interface_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )
