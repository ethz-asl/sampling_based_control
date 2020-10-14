/*!
 * @file     dimensions.h
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

/**
 * JOINT_DIMENSION: equal to the number of joints
 * INPUT_DIMENSION: the velocity command for each joint
 * INPUT_INTEGRAL_DIMENSION: integral control requires an additional state
 * STATE_DIMENSION: the combination of the dynamical system state
 * given by joint velocities and positions and the integral of the velocity input
 * REFERENCE_DIMENSION: composed of end effector pose (translation + quaternion) and obstacle cartesian position
 */
enum PandaDim: char {
  JOINT_DIMENSION = 7,
  INPUT_DIMENSION = 7,
  INPUT_INTEGRAL_DIMENSION = 7,
  STATE_DIMENSION = 21,
  REFERENCE_DIMENSION = 10
};
