cmake_minimum_required(VERSION 3.10)
project(mppi_pinocchio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
 include
 ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED 
  src/model.cpp 
  src/ros_conversions.cpp)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} 
  EXPORT ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


ament_export_dependencies(Eigen3)
ament_export_dependencies(pinocchio)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
