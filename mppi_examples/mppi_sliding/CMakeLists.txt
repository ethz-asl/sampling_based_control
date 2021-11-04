cmake_minimum_required(VERSION 3.0.2)
project(mppi_sliding)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_WITH_OGRE "Build the ogre additional utilities and examples." OFF)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        mppi_pinocchio
        roslib
        roscpp
        manipulation_msgs
        tf2_ros
        signal_logger
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS raisim)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/params/gains.cpp
        src/params/dynamics_params.cpp
        src/params/cost_params.cpp
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp
        src/reference_scheduler.cpp
        src/dynamics_ros.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES} yaml-cpp)

add_executable(manipulation_control src/nodes/panda_control.cpp)
target_link_libraries(manipulation_control ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(manipulation_control ${PROJECT_NAME})

add_executable(manipulation_raisim src/tests/panda_raisim.cpp)
target_link_libraries(manipulation_raisim ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(manipulation_raisim ${PROJECT_NAME})

add_executable(reference_parsing_test src/tests/reference_parsing.cpp)
target_link_libraries(reference_parsing_test ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(reference_parsing_test ${PROJECT_NAME})

if (BUILD_WITH_OGRE)
  find_package(raisimOgre CONFIG REQUIRED)
  add_executable(sim_ogre src/tests/sim.cpp)
  target_link_libraries(sim_ogre PUBLIC raisim::raisimOgre)
  add_dependencies(sim_ogre ${PROJECT_NAME})

  add_executable(raisim_rendering_node src/nodes/raisim_rendering_node.cpp)
  target_link_libraries(raisim_rendering_node PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES} raisim::raisimOgre)
  add_dependencies(raisim_rendering_node ${PROJECT_NAME})
endif()

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
