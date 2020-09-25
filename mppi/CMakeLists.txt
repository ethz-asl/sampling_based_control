cmake_minimum_required(VERSION 3.10)
project(mppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED 3.3)
find_package(gram_savitzky_golay REQUIRED)
find_package(yaml-cpp REQUIRED)

# GUI dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(X11 REQUIRED)

# Imgui
set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/submodules/imgui)
set(IMGUI_EXAMPLES_DIR ${IMGUI_DIR}/examples)
set(IMGUI_INCLUDE_DIRS ${IMGUI_DIR} ${IMGUI_EXAMPLES_DIR})

file(GLOB IMGUI_HDRS ${IMGUI_DIR}/*.h)
set(IMGUI_HDRS ${IMGUI_HDRS} ${IMGUI_EXAMPLES_DIR}/imgui_impl_glfw.h ${IMGUI_EXAMPLES_DIR}/imgui_impl_opengl2.h)

file(GLOB IMGUI_SRCS ${IMGUI_DIR}/*.cpp)
set(IMGUI_SRCS ${IMGUI_SRCS}  ${IMGUI_EXAMPLES_DIR}/imgui_impl_glfw.cpp ${IMGUI_EXAMPLES_DIR}/imgui_impl_opengl2.cpp)

# Implot
set(IMPLOT_DIR ${PROJECT_SOURCE_DIR}/submodules/implot)
set(IMPLOT_INCLUDE_DIR ${IMPLOT_DIR})
file(GLOB IMPLOT_HDRS ${IMPLOT_DIR}/*.h)
file(GLOB IMPLOT_SRCS ${IMPLOT_DIR}/*.cpp)

# Gui vars
set(GUI_LIBRARIES ${OPENGL_LIBRARIES} glfw ${GLFW_LIBRARY_DIRS})
set(GUI_INCLUDE_DIRS
        ${OPENGL_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        ${IMGUI_INCLUDE_DIRS}
        ${IMPLOT_INCLUDE_DIR})


set(COMMON_LIBRARIES  ${catkin_LIBRARIES} yaml-cpp gram_savitzky_golay::gram_savitzky_golay)
set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${gram_savitzky_golay_INCLUDE_DIRS}
        ${yaml_cpp_INCLUDE_DIRS})



find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs roscpp)
catkin_package(
        LIBRARIES ${PROJECT_NAME} gui
        INCLUDE_DIRS include ${GUI_INCLUDE_DIRS}
        CATKIN_DEPENDS
        DEPENDS yaml-cpp)

include_directories(include ${SYSTEM_INCLUDE_DIRECTORIES} ${GUI_INCLUDE_DIRS})

add_library(gui
        src/gui/gui.cpp
        #${IMGUI_HDRS}
        ${IMGUI_SRCS}
        #${IMPLOT_HDRS}
        ${IMPLOT_SRCS})

target_link_libraries(gui ${GUI_LIBRARIES})
target_link_libraries(gui
        ${CMAKE_THREAD_LIBS_INIT}
        ${X11_LIBRARIES}
        ${CMAKE_DL_LIBS}
        )

# Libraries
add_library(${PROJECT_NAME}
        src/controller/mppi.cpp
        src/controller/rollout.cpp
        src/dynamics/dynamics_base.cpp
        src/cost/cost_base.cpp
        src/solver_config.cpp
        src/filters/savgol_filter.cpp
        src/visualization/path_integral_visualizer.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LIBRARIES} gui )
target_compile_options(${PROJECT_NAME} PUBLIC -DDEBUG_PRINT)
add_dependencies(${PROJECT_NAME} gui)

## Test Visual debugger
#add_executable(mppi_visualizer_double_integrator
#        unittest/double_integrator.cpp
#        unittest/mppi_visualizer.cpp)
#target_link_libraries(mppi_visualizer_double_integrator ${PROJECT_NAME} ${COMMON_LIBRARIES})
#add_dependencies(mppi_visualizer_double_integrator ${PROJECT_NAME})


## Testing
#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(mppi_methods_test unittest/mppi_methods.cpp)
#    target_link_libraries(mppi_methods_test ${PROJECT_NAME} ${catkin_LIBRARIES})

#    catkin_add_gtest(multivariate_test unittest/multivariate_normal_eigen_test.cpp)
#    target_link_libraries(multivariate_test ${PROJECT_NAME})
#
#    catkin_add_gtest(solver_config unittest/solver_config.cpp)
#    target_link_libraries(solver_config ${PROJECT_NAME} ${COMMON_LIBRARIES})
#
#    catkin_add_gtest(thread_pool unittest/thread_pool.cpp)
#    target_link_libraries(solver_config ${PROJECT_NAME} ${COMMON_LIBRARIES})
#
#    catkin_add_gtest(logging unittest/logging.cpp)
#    target_link_libraries(logging ${PROJECT_NAME} ${COMMON_LIBRARIES})
#endif()

### Test the filter
#find_package(gram_savitzky_golay REQUIRED)
#find_package(rosbag REQUIRED)
#find_package(std_msgs REQUIRED)
#add_executable(savgol_filter unittest/savgol_filter.cpp)
#target_link_libraries(savgol_filter PUBLIC gram_savitzky_golay::gram_savitzky_golay)

