cmake_minimum_required(VERSION 3.10)
project(mppi_omav_raisim)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        roslib
        roscpp
        tf2_ros
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
        )
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES})

add_executable(omav_control src/nodes/omav_control.cpp)
target_link_libraries(omav_control ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(omav_control ${PROJECT_NAME})

add_executable(omav_raisim src/tests/omav_raisim.cpp)
target_link_libraries(omav_raisim ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(omav_raisim ${PROJECT_NAME})

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
