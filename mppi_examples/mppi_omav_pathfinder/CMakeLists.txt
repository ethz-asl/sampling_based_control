cmake_minimum_required(VERSION 3.10)
project(mppi_omav_pathfinder)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        LIBRARIES mppi_omav_pathfinder
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRAIRES})

add_executable(omav_pathfinder_control src/nodes/omav_pathfinder_control.cpp)
target_link_libraries(omav_pathfinder_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(omav_pathfinder_control ${PROJECT_NAME})

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
