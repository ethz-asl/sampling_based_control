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
 * INPUT_DIMENSION: the velocity command for each joint and the gripper position
 * command STATE_DIMENSION: joints velocity, position, position_desired
 * (integral of velocity) + gripper position and velocity REFERENCE_DIMENSION:
 * ee pose (translation + quaternion) and door opening value (revolute joint)
 */

//  state x
//  [ q_arm, q_gripper, q_dot_arm, q_dot_gripper, door_p, door_v, q_des_arm,
//  q_des_gripper] [   7  ,     2    ,     7    ,       2      ,    1  ,    1  ,
//  7    ,       2       ]

enum PandaDim : char {
  GRIPPER_DIMENSION = 2,  // position of the gripper fingers (2)
  DOOR_DIMENSION = 1,
  ARM_DIMENSION = 7,                                    // arm only joints
  JOINT_DIMENSION = ARM_DIMENSION + GRIPPER_DIMENSION,  // overall joints
  REFERENCE_POSE_DIMENSION = 7,
  REFERENCE_OBSTACLE = 3,
  REFERENCE_DOOR_DIMENSION = 1,
  REFERENCE_FLAG = 1,  // trigger some costs components
  CONTACT_STATE = 1,

  STATE_DIMENSION = 2 * JOINT_DIMENSION + 2 * DOOR_DIMENSION +
                    CONTACT_STATE,      // q, q_dot + door, door_dot, contact
  INPUT_DIMENSION = ARM_DIMENSION + 1,  // arm joints velocity and gripper cmd
  REFERENCE_DIMENSION = REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE +
                        REFERENCE_DOOR_DIMENSION + REFERENCE_FLAG
};
