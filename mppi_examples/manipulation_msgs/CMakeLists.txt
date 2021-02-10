cmake_minimum_required(VERSION 3.0.2)
project(manipulation_msgs)

set(CMAKE_CXX_STANDARD 17)
set(PACKAGE_DEPENDENCIES geometry_msgs
        std_msgs
        sensor_msgs
        message_generation
        )
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Eigen3 REQUIRED 3.3)

add_message_files(
        DIRECTORY msg
        FILES State.msg Input.msg InputState.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PROJECT_DEPENDENCIES} message_runtime
        DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/conversions.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(conversions_test unittest/conversions_test.cpp)
add_dependencies(conversions_test ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(conversions_test ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
