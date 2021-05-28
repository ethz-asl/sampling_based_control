cmake_minimum_required(VERSION 3.10)
project(mppi_panda_mobile)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED 3.3)

list(APPEND AMENT_PACKAGES sensor_msgs nav_msgs visualization_msgs std_msgs rclcpp mppi mppi_ros mppi_pinocchio tf2 tf2_ros)

foreach (pkg ${AMENT_PACKAGES})
    find_package(${pkg} REQUIRED)
endforeach ()

include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGES})

add_executable(panda_mobile_control src/nodes/panda_mobile_control.cpp)
target_link_libraries(panda_mobile_control ${PROJECT_NAME})
add_dependencies(panda_mobile_control ${PROJECT_NAME})
ament_target_dependencies(panda_mobile_control ${AMENT_PACKAGES})

add_executable(controller_node src/nodes/controller_node.cpp)
target_link_libraries(controller_node ${PROJECT_NAME})
add_dependencies(controller_node ${PROJECT_NAME})
ament_target_dependencies(controller_node ${AMENT_PACKAGES})

install(TARGETS panda_mobile_control controller_node
  DESTINATION lib/${PROJECT_NAME})
  
install(PROGRAMS scripts/target_generator.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        launch config resources
        DESTINATION share/${PROJECT_NAME}/
        )

ament_export_dependencies(Eigen3)
foreach (pkg ${AMENT_PACKAGES})
    ament_export_dependencies(${pkg} REQUIRED)
endforeach ()
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)


ament_package()
