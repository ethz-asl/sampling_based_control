cmake_minimum_required(VERSION 3.10)
project(mppi_ros)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)

set(PACKAGE_DEPENDENCIES
        rosbag
        mppi
        sensor_msgs
        std_msgs
        roscpp
        any_worker
        param_io)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES} message_generation)
find_package(gram_savitzky_golay)

# Messages
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg
        FILES Config.msg Rollout.msg Data.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

set(COMMON_LIBRARIES ${catkin_LIBRARIES})
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(include SYSTEM ${SYSTEM_INCLUDE_DIRECTORIES})


# Libraries
add_library(${PROJECT_NAME}
        src/controller_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

## Testing
if (CATKIN_ENABLE_TESTING)
    #    catkin_add_gtest()
    #    target_link_libraries()

endif ()