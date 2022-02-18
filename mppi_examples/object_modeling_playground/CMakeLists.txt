cmake_minimum_required(VERSION 3.0.2)
project(object_modeling_playground)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED 3.3)
find_package(raisim CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        manipulation_msgs
        keypoint_msgs
        roscpp
        roslib
        roscpp
        tf2_ros
        pcl_ros
        interactive_markers
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})


include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME}
        src/object.cpp
        src/mug.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)


add_executable(primitive_modeling src/main.cpp)
target_link_libraries(primitive_modeling ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(primitive_modeling ${PROJECT_NAME})

catkin_install_python(PROGRAMS scripts/interactive_marker.py  scripts/cylinder_fitting_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

