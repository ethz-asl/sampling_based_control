catkin_add_gtest(test_ros_conversions src/test_ros_conversions.cpp)
target_link_libraries(test_ros_conversions ${PROJECT_NAME} ${catkin_LIBRARIES})