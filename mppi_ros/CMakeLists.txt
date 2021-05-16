cmake_minimum_required(VERSION 3.10)
project(mppi_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED 3.3)
find_package(mppi REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages
set(msg_files
  "msg/Array.msg"
  "msg/Config.msg"
  "msg/Data.msg"
  "msg/Rollout.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  ${msg_files})


set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
include_directories(include SYSTEM ${SYSTEM_INCLUDE_DIRECTORIES})

add_library(${PROJECT_NAME}
  src/controller_interface.cpp
  src/conversions.cpp
  src/threading/Rate.cpp
  src/threading/Worker.cpp
  src/threading/WorkerManager.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  mppi
  sensor_msgs
  std_msgs
  rclcpp
)

# This is needed since we are using msgs in the same package where they are defined
rosidl_target_interfaces(${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

install(
        TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(PROGRAMS
  scripts/plot_experiment.py
  scripts/record_data.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(Eigen3)
ament_export_dependencies(mppi)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
