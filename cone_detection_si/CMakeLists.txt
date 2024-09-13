cmake_minimum_required(VERSION 3.0.2)
project(cone_detection_si)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
  tf
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

add_executable(${PROJECT_NAME} src/cone_detection_si.cpp src/cone_detection_si_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )
