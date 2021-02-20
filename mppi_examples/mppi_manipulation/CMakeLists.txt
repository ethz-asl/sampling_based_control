cmake_minimum_required(VERSION 3.10)
project(mppi_manipulation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_WITH_OGRE "Build the ogre additional utilities and examples." OFF)

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(orocos_kdl)

set(PACKAGE_DEPENDENCIES
        sensor_msgs
        std_msgs
        roscpp
        mppi
        mppi_ros
        roslib
        roscpp
        robot_control
        controller_interface
        franka_hw
        hardware_interface
        franka_example_controllers
        kdl_parser
        manipulation_msgs
        gazebo_msgs
        )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
        DEPENDS pinocchio raisim Franka orocos_kdl)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/gains.cpp
        src/dynamics.cpp
        src/cost.cpp
        src/controller_interface.cpp
        src/dynamics_ros.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_executable(manipulation_control src/nodes/panda_control.cpp)
target_link_libraries(manipulation_control ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(manipulation_control ${PROJECT_NAME})

add_executable(manipulation_raisim src/tests/panda_raisim.cpp)
target_link_libraries(manipulation_raisim ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(manipulation_raisim ${PROJECT_NAME})

if (BUILD_WITH_OGRE)
  find_package(raisimOgre CONFIG REQUIRED)
  add_executable(sim_ogre src/tests/sim.cpp)
  target_link_libraries(sim_ogre PUBLIC raisim::raisimOgre)
  add_dependencies(sim_ogre ${PROJECT_NAME})

  add_executable(raisim_rendering_node src/nodes/raisim_rendering_node.cpp)
  target_link_libraries(raisim_rendering_node PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES} raisim::raisimOgre)
  add_dependencies(raisim_rendering_node ${PROJECT_NAME})
endif()


add_library(${PROJECT_NAME}_controller
        src/ros/royal_panda_controller_ros.cpp)
target_include_directories(${PROJECT_NAME}_controller PUBLIC ${catkin_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME} ${catkin_LIBRARIES} ${Franka_LIBRARIES})
add_dependencies(${PROJECT_NAME}_controller ${PROJECT_NAME})

add_library(controller_test src/ros/controller_test.cpp)
target_include_directories(controller_test PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(controller_test ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(controller_test ${PROJECT_NAME})

add_executable(controller_node src/nodes/controller_node.cpp)
target_link_libraries(controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(controller_test ${PROJECT_NAME})

add_executable(base_odometry_dummy_node src/test/base_odometry_dummy_node.cpp)
target_link_libraries(base_odometry_dummy_node ${catkin_LIBRARIES})

add_executable(joint_state_filter_node src/ros/joint_state_filter_node.cpp)
target_include_directories(joint_state_filter_node PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(joint_state_filter_node ${catkin_LIBRARIES})

add_executable(vicon_emulator_node src/ros/vicon_emulator_node.cpp)
target_include_directories(vicon_emulator_node PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(vicon_emulator_node ${catkin_LIBRARIES})

add_executable(state_observer_node src/ros/state_observer.cpp src/ros/state_observer_node.cpp)
target_include_directories(state_observer_node PUBLIC include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(state_observer_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARY_DIRS})

#set(CATKIN_ENABLE_TESTING True)
#if(CATKIN_ENABLE_TESTING)
#    add_executable(base_twist_publisher  src/test/base_twist_publisher.cpp)
#    target_link_libraries(base_twist_publisher ${catkin_LIBRARIES})
#
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(state_assembler_test test/state_assembler.test src/test/state_assembler_node.cpp)
#    target_link_libraries(state_assembler_test ${catkin_LIBRARIES} ${Franka_LIBRARIES})
#endif()

install(DIRECTORY launch resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(PROGRAMS scripts/unpause_physics.py scripts/demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )