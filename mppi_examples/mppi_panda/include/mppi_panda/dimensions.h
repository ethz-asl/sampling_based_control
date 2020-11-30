/*!
 * @file     dimensions.h
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

enum PandaDim {
  STATE_DIMENSION = 14,
  INPUT_DIMENSION = 7,
  REFERENCE_DIMENSION = 10  // ee pose + obstacle position
};
