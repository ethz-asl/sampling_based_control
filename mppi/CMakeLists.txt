cmake_minimum_required(VERSION 3.10)
project(mppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED 3.3)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

set(COMMON_LIBRARIES ${catkin_LIBRARIES} yaml-cpp)
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}${yaml_cpp_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs roscpp)
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        DEPENDS yaml-cpp)

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})

# Third party
add_library(gram_savitzky_golay
        src/filters/gram_savitzky_golay.cpp)

# Libraries
add_library(${PROJECT_NAME}
        src/controller/mppi.cpp
        src/controller/rollout.cpp
        src/dynamics/dynamics_base.cpp
        src/cost/cost_base.cpp
        src/solver_config.cpp
        src/filters/savgol_filter.cpp
        src/tree/tree_manager.cpp
        src/tree/node.cpp
        src/experts/expert.cpp
        src/utils/timer.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LIBRARIES} gram_savitzky_golay)
add_dependencies(${PROJECT_NAME} gram_savitzky_golay)
target_compile_options(${PROJECT_NAME} PUBLIC -DDEBUG_PRINT)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

if (CATKIN_ENABLE_TESTING)
    find_package(PythonLibs 2.7 REQUIRED)
    catkin_add_gtest(filter_test unittest/filter_test.cpp)
    target_include_directories(filter_test PRIVATE ${PYTHON_INCLUDE_DIRS})
    target_link_libraries(filter_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

endif ()


