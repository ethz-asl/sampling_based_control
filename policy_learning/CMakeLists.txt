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
        mppi_pinocchio
        actionlib_msgs
        sensor_msgs
        geometry_msgs)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

add_action_files(
        DIRECTORY actions
        FILES collect_rollout.action
)

generate_messages(
        DEPENDENCIES actionlib_msgs sensor_msgs geometry_msgs
)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${TORCH_LIBRARIES} HighFive)
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})
add_library(${PROJECT_NAME}
        src/panda_expert.cpp
        src/pole_cart_expert.cpp
        src/torch_script_policy.cpp
        src/hdf5_dataset.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -DDEBUG_PRINT)
