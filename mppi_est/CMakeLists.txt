cmake_minimum_required(VERSION 3.10)
project(mppi_est)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


find_package(Eigen3 REQUIRED 3.3)
find_package(catkin REQUIRED COMPONENTS )

set(COMMON_LIBRARIES ${catkin_LIBRARIES})
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        DEPENDS)

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES})

# Libraries
add_library(${PROJECT_NAME}
        src/model.cpp
        src/model_particle_filter.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -DDEBUG_PRINT)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(point_mass_test unittest/point_mass.cpp unittest/point_mass_test.cpp)
    target_include_directories(point_mass_test PRIVATE unittest include ${SYSTEM_INCLUDE_DIRS})
    target_link_libraries(point_mass_test ${PROJECT_NAME} ${catkin_LIBRARIES})

    catkin_add_gtest(pole_cart_test unittest/pole_cart.cpp unittest/pole_cart_test.cpp)
    target_include_directories(pole_cart_test PRIVATE unittest include ${SYSTEM_INCLUDE_DIRS})
    target_link_libraries(pole_cart_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()


