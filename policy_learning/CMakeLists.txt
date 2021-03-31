cmake_minimum_required(VERSION 3.10)
project(policy_learning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED 3.3)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(HighFive REQUIRED)

set(PACKAGE_DEPENDENCIES
        mppi
        roscpp)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${TORCH_LIBRARIES} HighFive)
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})
add_library(${PROJECT_NAME}
        src/offline_pytorch_expert.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -DDEBUG_PRINT)


