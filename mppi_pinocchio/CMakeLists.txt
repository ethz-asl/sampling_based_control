cmake_minimum_required(VERSION 3.0.2)
project(mppi_pinocchio)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)

set(PACKAGE_DEPENDENCIES geometry_msgs)
find_package(catkin REQUIRED ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS pinocchio
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/model.cpp src/ros_conversions.cpp src/ros_conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pinocchio::pinocchio)
