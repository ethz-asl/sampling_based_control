cmake_minimum_required(VERSION 3.10)
project(mppi_omav_interaction)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)
option(BUILD_RAISIM_VISUALIZATION "Build the raisim visualization server." OFF)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)

set(PACKAGE_DEPENDENCIES
        std_msgs
        roscpp
        message_filters
        mppi
        mppi_ros
        mppi_pinocchio
        roslib
        roscpp
        tf2_geometry_msgs
        tf2_ros
        mav_msgs
        tf
        eigen_conversions
        kdl_parser
        dynamic_reconfigure
        geometry_msgs
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

generate_dynamic_reconfigure_options(cfg/mppi_omav_reference.cfg cfg/mppi_omav_cost_shelf.cfg cfg/mppi_omav_cost_valve.cfg cfg/mppi_settings.cfg)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS raisim)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


if(BUILD_RAISIM_VISUALIZATION)
        add_compile_definitions(RAISIM_VISUALIZATION_SERVER)
endif()

add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost_shelf.cpp
        src/cost_valve.cpp
        src/controller_interface.cpp
        src/reference_pose_publisher.cpp
        src/ros_conversions.cpp
        src/state_observer.cpp
        src/state_observer_valve.cpp
        )
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(omav_interaction_control_node src/nodes/omav_interaction_control_node.cpp)
target_link_libraries(omav_interaction_control_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(omav_interaction_control_node ${PROJECT_NAME}_gencfg)

add_executable(reference_pose_publisher_node src/nodes/reference_pose_publisher_node.cpp)
target_link_libraries(reference_pose_publisher_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(reference_pose_publisher_node ${PROJECT_NAME}_gencfg)

add_executable(object_pose_mock_publisher_node src/nodes/object_pose_mock_publisher_node.cpp)
target_link_libraries(object_pose_mock_publisher_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(state_observer_node src/nodes/state_observer_node.cpp)
target_link_libraries(state_observer_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# Unit Tests
if(CATKIN_ENABLE_TESTING)
        add_subdirectory(tests)
endif(CATKIN_ENABLE_TESTING)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

install(DIRECTORY launch resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/button.py scripts/linear_drawer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
