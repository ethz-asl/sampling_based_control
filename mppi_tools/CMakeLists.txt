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
  LIBRARIES mppi_controller_gui mppi_model_tracking_controller
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS
)


###  Build 
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Imgui
set(IMGUI_DIR submodules/imgui)
set(IMGUI_SRC ${IMGUI_DIR}/imgui.cpp
              ${IMGUI_DIR}/imgui_demo.cpp
              ${IMGUI_DIR}/imgui_draw.cpp
              ${IMGUI_DIR}/imgui_tables.cpp
              ${IMGUI_DIR}/imgui_widgets.cpp
              ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
              ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp)
set(IMGUI_INCLUDE_DIRS ${IMGUI_DIR} ${IMGUI_DIR}/backends)

# Implot
set(IMPLOT_DIR submodules/implot)
set(IMPLOT_SRC ${IMPLOT_DIR}/implot.cpp
               ${IMPLOT_DIR}/implot_demo.cpp
               ${IMPLOT_DIR}/implot_items.cpp)
set(IMPLOT_INCLUDE_DIRS ${IMPLOT_DIR})

add_library(mppi_controller_gui src/control_gui.cpp ${IMGUI_SRC} ${IMPLOT_SRC})
target_link_libraries(mppi_controller_gui PUBLIC ${catkin_LIBRARIES} glfw OpenGL::GL)
target_include_directories(mppi_controller_gui PRIVATE ${IMGUI_INCLUDE_DIRS} ${IMPLOT_INCLUDE_DIRS})

add_library(mppi_model_tracking_controller src/model_tracking_controller.cpp)
target_link_libraries(mppi_model_tracking_controller PUBLIC ${catkin_LIBRARIES} mppi_controller_gui)
add_dependencies(mppi_model_tracking_controller mppi_controller_gui)

###  Build
add_executable(vtest unittest/control_gui_test.cpp)
target_link_libraries(vtest mppi_controller_gui ${catkin_LIBRARIES})
add_dependencies(vtest mppi_controller_gui)

#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(visual_debugger_test unittest/visual_debugger_test.cpp)
#    target_link_libraries(visual_debugger_test ${PROJECT_NAME} ${catkin_LIBRARIES})
#    add_dependencies(visual_debugger_test ${PROJECT_NAME})
#endif ()

###  Install 
install(TARGETS mppi_model_tracking_controller mppi_controller_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
