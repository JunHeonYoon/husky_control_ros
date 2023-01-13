cmake_minimum_required(VERSION 3.0.2)
project(husky_control_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# ---------------------------Headers and Class---------------------------------
set(SRCS_controller
    ${CMAKE_CURRENT_SOURCE_DIR}/src/husky_controller.cpp
)
set(HEADS_controller
    ${CMAKE_CURRENT_SOURCE_DIR}/include/husky_controller.h
)
add_library(vrep_husky_control ${SRCS_controller} ${HEADS_controller})
# ----------------------------Headers and Class End------------------------------


catkin_package()


# ------------------------------Excute Node-------------------------------------------------
add_executable(test_node_vrep src/test_main_vrep.cpp)
target_link_libraries(test_node_vrep vrep_husky_control ${catkin_LIBRARIES})
# ------------------------------Excute Node End-------------------------------------------------
