cmake_minimum_required(VERSION 3.10)
project(mppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED 3.3)
find_package(yaml-cpp REQUIRED)

set(COMMON_LIBRARIES yaml-cpp)
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS})

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})

# Third party
add_library(gram_savitzky_golay
        src/filters/gram_savitzky_golay.cpp)

# Libraries
add_library(${PROJECT_NAME} SHARED
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

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} gram_savitzky_golay 
  EXPORT ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if (BUILD_TESTING)
    find_package(PythonLibs 2.7 REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(filter_test unittest/filter_test.cpp)
    target_include_directories(filter_test PRIVATE ${PYTHON_INCLUDE_DIRS})
    target_link_libraries(filter_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

endif ()

ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_package()