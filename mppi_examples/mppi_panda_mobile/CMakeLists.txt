cmake_minimum_required(VERSION 3.10)
project(mppi_panda_mobile)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)

set(PACKAGE_DEPENDENCIES sensor_msgs std_msgs roscpp mppi roslib tf2 tf2_ros)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS pinocchio)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PINOCCHIO_LIBRARIES})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/ros_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRAIRES} ${PINOCCHIO_LIBRARIES})

add_executable(panda_mobile_free src/nodes/panda_free.cpp)
target_link_libraries(panda_mobile_free ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(panda_mobile_free ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(panda_mobile_control src/nodes/panda_control.cpp)
target_link_libraries(panda_mobile_control ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(panda_mobile_control ${PROJECT_NAME})

install(DIRECTORY launch resources
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
