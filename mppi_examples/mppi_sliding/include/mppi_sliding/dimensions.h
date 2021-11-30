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
    //OBJECT_DIMENSION = 1,
    OBJECT_DIMENSION = 7,  
    CONTACT_STATE = 1,
    TORQUE_DIMENSION = BASE_DIMENSION + ARM_DIMENSION + GRIPPER_DIMENSION,

    ARM_GRIPPER_DIM = ARM_DIMENSION + GRIPPER_DIMENSION,
    BASE_ARM_GRIPPER_DIM = BASE_DIMENSION + ARM_GRIPPER_DIM,

    //STATE_DIMENSION = BASE_ARM_GRIPPER_DIM * 2 + OBJECT_DIMENSION * 2 + CONTACT_STATE + TORQUE_DIMENSION,
    STATE_DIMENSION = BASE_ARM_GRIPPER_DIM * 2 + OBJECT_DIMENSION * 2 + CONTACT_STATE,
    INPUT_DIMENSION = BASE_ARM_GRIPPER_DIM - 1, // mimic-joint for gripper

    REFERENCE_POSE_DIMENSION = 7,
    REFERENCE_OBSTACLE = 3,
    //REFERENCE_OBJECT_DIMENSION = 1,  // door handle
    REFERENCE_OBJECT_DIMENSION = 7,    // mug pushing
    REFERENCE_FLAG = 1,  // trigger some costs components

    REFERENCE_DIMENSION = REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE +
                          REFERENCE_OBJECT_DIMENSION + REFERENCE_FLAG
};

