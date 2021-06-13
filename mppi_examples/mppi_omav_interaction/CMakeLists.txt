cmake_minimum_required(VERSION 3.10)
project(mppi_omav_interaction)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)
find_package(catkin REQUIRED dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/mppi_omav_reference.cfg cfg/mppi_omav_cost.cfg)

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
        tf
        eigen_conversions
        kdl_parser
        dynamic_reconfigure
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
        src/state_observer.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES})

add_executable(omav_interaction_control src/nodes/omav_interaction_control.cpp)
target_link_libraries(omav_interaction_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(omav_interaction_control ${PROJECT_NAME})
add_dependencies(omav_interaction_control ${PROJECT_NAME}_gencfg)

add_executable(state_observer_node src/nodes/state_observer_node.cpp)
target_link_libraries(state_observer_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

install(DIRECTORY launch resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/button.py scripts/linear_drawer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
