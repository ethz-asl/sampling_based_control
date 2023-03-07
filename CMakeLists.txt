cmake_minimum_required(VERSION 3.10)
project(sampling_based_control)

add_subdirectory(./mppi)
add_subdirectory(./mppi_ros)
add_subdirectory(./mppi_pinocchio)
add_subdirectory(./mppi_examples/mppi_omav_interaction)