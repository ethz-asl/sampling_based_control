cmake_minimum_required(VERSION 3.10)
project(mppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED 3.3)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs roscpp)
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        DEPENDS yaml-cpp)

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})

file(GLOB_RECURSE SRCS src/*.cpp)
add_library(${PROJECT_NAME} ${SRCS} src/policies/spline_policy.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} yaml-cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

option(PYTHON_BINDINGS "Build the python bindings" ON)
if (${PYTHON_BINDINGS})
    add_subdirectory(../submodules/pybind11 ../submodules/pybind11)

    pybind11_add_module(pymppi python/python.cpp)
    target_link_libraries(pymppi PUBLIC ${PROJECT_NAME})
    add_dependencies(pymppi ${PROJECT_NAME})
endif()

if (CATKIN_ENABLE_TESTING)
    find_package(PythonLibs 2.7 REQUIRED)
    catkin_add_gtest(spline_test unittest/spline.cpp)
    target_link_libraries(spline_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()
