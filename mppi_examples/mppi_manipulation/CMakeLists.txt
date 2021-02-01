cmake_minimum_required(VERSION 3.10)
project(mppi_manipulation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(raisimOgre CONFIG REQUIRED)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        roslib
        roscpp
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS pinocchio raisim) 

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp
        src/dynamics_ros.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_executable(manipulation_control src/nodes/panda_control.cpp)
target_link_libraries(manipulation_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(manipulation_control ${PROJECT_NAME})

add_executable(manipulation_raisim src/tests/panda_raisim.cpp)
target_link_libraries(manipulation_raisim ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(manipulation_raisim ${PROJECT_NAME})

add_executable(sim_ogre src/tests/sim.cpp)
target_link_libraries(sim_ogre PUBLIC raisim::raisimOgre)
add_dependencies(sim_ogre ${PROJECT_NAME})

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
