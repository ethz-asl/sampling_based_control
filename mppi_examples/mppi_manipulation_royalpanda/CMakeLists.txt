cmake_minimum_required(VERSION 3.0.2)
project(mppi_manipulation_royalpanda)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)
find_package(Franka 0.7.0 REQUIRED)
find_package(orocos_kdl)

set(PACKAGE_DEPENDENCIES
    sensor_msgs
    std_msgs
    roscpp
    mppi_ros
    mppi_manipulation
    roslib
    roscpp
    mppi_pinocchio
    controller_interface
    controller_manager
    franka_hw
    hardware_interface
    kdl_parser
    manipulation_msgs
    eigen_conversions
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_controller
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS Franka orocos_kdl
)

include_directories(include ${catkin_INCLUDE_DIRS})

### Simulation
add_executable(simulation_node src/simulation.cpp src/nodes/simulation_node.cpp)
target_link_libraries(simulation_node ${catkin_LIBRARIES})

### Ros controller
add_library(manipulation_controller src/controller.cpp)
target_include_directories(manipulation_controller PUBLIC ${catkin_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS})
target_link_libraries(manipulation_controller ${catkin_LIBRARIES} ${Franka_LIBRARIES})

#### State observer node
add_executable(state_observer_node src/state_observer.cpp src/nodes/state_observer_node.cpp)
target_include_directories(state_observer_node PUBLIC include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(state_observer_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARY_DIRS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)