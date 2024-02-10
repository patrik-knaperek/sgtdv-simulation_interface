cmake_minimum_required(VERSION 3.0.2)
project(control_si)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    fsd_common_msgs
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

add_executable(${PROJECT_NAME} src/control_si_node.cpp src/control_si.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)