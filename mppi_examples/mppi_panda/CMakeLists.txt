cmake_minimum_required(VERSION 3.10)
project(mppi_panda)

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
    roscpp
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES mppi_panda
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(panda_control src/nodes/panda_control.cpp)
target_link_libraries(panda_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(panda_control ${PROJECT_NAME})

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/interactive_marker.py scripts/target_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)