cmake_minimum_required(VERSION 3.10)
project(mppi_panda_mobile)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        mppi_pinocchio
        roslib
        tf2
        tf2_ros
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS )

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRAIRES})

add_executable(panda_mobile_control src/nodes/panda_mobile_control.cpp)
target_link_libraries(panda_mobile_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(panda_mobile_control ${PROJECT_NAME})

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(PROGRAMS scripts/target_generator.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )