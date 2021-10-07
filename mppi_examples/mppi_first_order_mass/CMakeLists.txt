cmake_minimum_required(VERSION 3.10)
project(mppi_first_order_mass)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        mppi_tools)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        LIBRARIES mppi_first_order_mass
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRAIRES})

add_executable(mass_control src/nodes/mass_control.cpp)
target_link_libraries(mass_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(mass_control ${PROJECT_NAME})

add_executable(model_tracking src/nodes/model_tracking_node.cpp src/model_tracking.cpp)
target_link_libraries(model_tracking ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(model_tracking ${PROJECT_NAME})


install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
