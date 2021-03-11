cmake_minimum_required(VERSION 3.0.2)
project(mppi_tools)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED 3.3)

set(PACKAGE_DEPENDENCIES mppi)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mppi
  DEPENDS
)


###  Build 
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/visual_debugger.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} glfw OpenGL::GL)

###  Build
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(visual_debugger_test unittest/visual_debugger_test.cpp)
    target_link_libraries(visual_debugger_test ${PROJECT_NAME} ${catkin_LIBRARIES})
    add_dependencies(visual_debugger_test ${PROJECT_NAME})
endif ()

###  Install 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
