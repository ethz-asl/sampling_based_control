/*!
 * @file     dimensions.h
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

enum PandaDim{
  STATE_DIMENSION = 14, // joint positions and velocities
  INPUT_DIMENSION = 7,  // joint velocities
  REFERENCE_DIMENSION = 10  // end effector pose (x, y, z + quaterionion) + obstacle position
};
