/*!
 * @file     dimensions.h
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

enum PandaDim : char {
  GRIPPER_DIMENSION = 2,  // position of the gripper fingers (2)
  ARM_DIMENSION = 7,      // arm only joints
  BASE_DIMENSION = 3,
  OBJECT_DIMENSION = 1,

  ARM_GRIPPER_DIM = ARM_DIMENSION + GRIPPER_DIMENSION,
  BASE_ARM_GRIPPER_DIM = BASE_DIMENSION + ARM_GRIPPER_DIM,

  REFERENCE_POSE_DIMENSION = 7,
  REFERENCE_OBSTACLE = 3,
  REFERENCE_OBJECT_DIMENSION = 1,
  REFERENCE_FLAG = 1,  // trigger some costs components
  CONTACT_STATE = 1,

  REFERENCE_DIMENSION = REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE +
                        REFERENCE_OBJECT_DIMENSION + REFERENCE_FLAG
};
