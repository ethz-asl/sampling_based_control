cmake_minimum_required(VERSION 3.10)
project(mppi_omav_interaction)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)

set(PACKAGE_DEPENDENCIES
        std_msgs
        roscpp
        mppi
        mppi_ros
        mppi_pinocchio
        roslib
        roscpp
        tf2_ros
        mav_msgs
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS raisim)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/dynamics_ros.cpp
        src/cost.cpp
        src/controller_interface.cpp
        src/ros_conversions.cpp
        )
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES})

add_executable(omav_interaction_control src/nodes/omav_interaction_control.cpp)
target_link_libraries(omav_interaction_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(omav_interaction_control ${PROJECT_NAME})


add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(PROGRAMS scripts/demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
