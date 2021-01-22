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
 * ee pose (translation + quaternion) and object opening value (revolute joint)
 */

//  state x
//  [ q_arm, q_gripper, q_dot_arm, q_dot_gripper, object_p, object_v, q_des_arm,
//  q_des_gripper] [   7  ,     2    ,     7    ,       2      ,    1  ,    1  ,
//  7    ,       2       ]

enum PandaDim : char {
  GRIPPER_DIMENSION = 2,  // position of the gripper fingers (2)
  ARM_DIMENSION = 7,                                    // arm only joints
  BASE_DIMENSION = 3,
  OBJECT_DIMENSION = 1,

  ARM_GRIPPER_DIM = ARM_DIMENSION + GRIPPER_DIMENSION,
  BASE_ARM_GRIPPER_DIM = BASE_DIMENSION + ARM_GRIPPER_DIM,

  JOINT_DIMENSION = ARM_DIMENSION + GRIPPER_DIMENSION,  // overall joints
  REFERENCE_POSE_DIMENSION = 7,
  REFERENCE_OBSTACLE = 3,
  REFERENCE_OBJECT_DIMENSION = 1,
  REFERENCE_FLAG = 1,  // trigger some costs components
  CONTACT_STATE = 1,

  STATE_DIMENSION = 2 * JOINT_DIMENSION + 2 * OBJECT_DIMENSION +
                    CONTACT_STATE,      // q, q_dot + object, object_dot, contact
  INPUT_DIMENSION = ARM_DIMENSION + 1,  // arm joints velocity and gripper cmd
  REFERENCE_DIMENSION = REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE +
                        REFERENCE_OBJECT_DIMENSION + REFERENCE_FLAG
};

namespace manipulation {
struct dimensions {
  explicit dimensions(bool _fixed_base=true) : fixed_base(_fixed_base) {
    if (fixed_base){
      joint = arm + gripper;
      state = 2*joint + 2 * obj + contact;
      input = arm + 1;

    }
    else{
      joint = arm + base + gripper;
      state = 2*joint + 2 * obj + contact;
      input = arm + base + 1;
    }
  }

  // dofs
  size_t gripper = 2;
  size_t arm = 7;
  size_t base = 3;
  size_t obj = 1;
  size_t contact = 1;

  // reference
  size_t pose = 7;
  size_t obst = 3;
  size_t flag = 1;
  size_t ref = pose + obst + flag + obj;

  bool fixed_base;
  size_t joint;
  size_t state;
  size_t input;
};
}