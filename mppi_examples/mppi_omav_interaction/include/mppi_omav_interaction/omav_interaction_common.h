#ifndef MPPI_OMAV_INTERACTION_OMAV_INTERACTION_COMMON_H
#define MPPI_OMAV_INTERACTION_OMAV_INTERACTION_COMMON_H

#include <string>

namespace omav_interaction {

namespace omav_state_description {
/**
 * @brief Full state vector of OMAV.
 */
enum OmavStateDescription {
  MAV_POSITION_X_WORLD,
  MAV_POSITION_Y_WORLD,
  MAV_POSITION_Z_WORLD,
  MAV_ORIENTATION_W_WORLD,
  MAV_ORIENTATION_X_WORLD,
  MAV_ORIENTATION_Y_WORLD,
  MAV_ORIENTATION_Z_WORLD,
  MAV_LINEAR_VELOCITY_X_WORLD,
  MAV_LINEAR_VELOCITY_Y_WORLD,
  MAV_LINEAR_VELOCITY_Z_WORLD,
  MAV_ANGULAR_VELOCITY_X_BODY,
  MAV_ANGULAR_VELOCITY_Y_BODY,
  MAV_ANGULAR_VELOCITY_Z_BODY,
  OBJECT_ORIENTATION,
  OBJECT_VELOCITY,
  INTERACTION_FORCE_X,
  INTERACTION_FORCE_Y,
  INTERACTION_FORCE_Z,
  CONTACT_STATE,
  MAV_POSITION_X_DESIRED_WORLD,
  MAV_POSITION_Y_DESIRED_WORLD,
  MAV_POSITION_Z_DESIRED_WORLD,
  MAV_ORIENTATION_W_DESIRED_WORLD,
  MAV_ORIENTATION_X_DESIRED_WORLD,
  MAV_ORIENTATION_Y_DESIRED_WORLD,
  MAV_ORIENTATION_Z_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_X_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_Y_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_Z_DESIRED_WORLD,
  MAV_ANGULAR_VELOCITY_X_DESIRED_BODY,
  MAV_ANGULAR_VELOCITY_Y_DESIRED_BODY,
  MAV_ANGULAR_VELOCITY_Z_DESIRED_BODY,
  SIZE_OMAV_STATE
};
}  // namespace omav_state_description

namespace omav_state_description_simulation {
/**
 * @brief Full extended state vector of OMAV with additional state of unwanted
 *        contact. This additional unwanted contact state is needed to
 *        calculate the cost for the simulated rollouts.
 */
enum OmavStateDescriptionSimulation {
  MAV_POSITION_X_WORLD,
  MAV_POSITION_Y_WORLD,
  MAV_POSITION_Z_WORLD,
  MAV_ORIENTATION_W_WORLD,
  MAV_ORIENTATION_X_WORLD,
  MAV_ORIENTATION_Y_WORLD,
  MAV_ORIENTATION_Z_WORLD,
  MAV_LINEAR_VELOCITY_X_WORLD,
  MAV_LINEAR_VELOCITY_Y_WORLD,
  MAV_LINEAR_VELOCITY_Z_WORLD,
  MAV_ANGULAR_VELOCITY_X_BODY,
  MAV_ANGULAR_VELOCITY_Y_BODY,
  MAV_ANGULAR_VELOCITY_Z_BODY,
  OBJECT_ORIENTATION,
  OBJECT_VELOCITY,
  INTERACTION_FORCE_X,
  INTERACTION_FORCE_Y,
  INTERACTION_FORCE_Z,
  CONTACT_STATE,
  MAV_POSITION_X_DESIRED_WORLD,
  MAV_POSITION_Y_DESIRED_WORLD,
  MAV_POSITION_Z_DESIRED_WORLD,
  MAV_ORIENTATION_W_DESIRED_WORLD,
  MAV_ORIENTATION_X_DESIRED_WORLD,
  MAV_ORIENTATION_Y_DESIRED_WORLD,
  MAV_ORIENTATION_Z_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_X_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_Y_DESIRED_WORLD,
  MAV_LINEAR_VELOCITY_Z_DESIRED_WORLD,
  MAV_ANGULAR_VELOCITY_X_DESIRED_BODY,
  MAV_ANGULAR_VELOCITY_Y_DESIRED_BODY,
  MAV_ANGULAR_VELOCITY_Z_DESIRED_BODY,
  UNWANTED_CONTACT,
  SIZE_OMAV_STATE_SIMULATION
};
}  // namespace omav_state_description_simulation

namespace control_input_description {
enum ControlInputDescription {
  MAV_LINEAR_ACCELERATION_X_DESIRED_WORLD,
  MAV_LINEAR_ACCELERATION_Y_DESIRED_WORLD,
  MAV_LINEAR_ACCELERATION_Z_DESIRED_WORLD,
  MAV_ANGULAR_ACCELERATION_X_DESIRED_BODY,
  MAV_ANGULAR_ACCELERATION_Y_DESIRED_BODY,
  MAV_ANGULAR_ACCELERATION_Z_DESIRED_BODY,
  SIZE_CONTROL_INPUT
};
}  // namespace control_input_description

namespace object_state_description {
/**
 * @brief Full state vector of the object (door, valve, etc.).
 */
enum ObjectStateDescription {
  OBJECT_ORIENTATION,
  OBJECT_VELOCITY,
  SIZE_OBJECT_STATE
};
}  // namespace object_state_description

namespace reference_description {
/**
 * @brief Reference trajectory vector.
 */
enum ReferenceDescription {
  MAV_GOAL_POSITION_X_WORLD,
  MAV_GOAL_POSITION_Y_WORLD,
  MAV_GOAL_POSITION_Z_WORLD,
  MAV_GOAL_ORIENTATION_W_WORLD,
  MAV_GOAL_ORIENTATION_X_WORLD,
  MAV_GOAL_ORIENTATION_Y_WORLD,
  MAV_GOAL_ORIENTATION_Z_WORLD,
  OBJECT_GOAL_ORIENTATION,
  INTERACTION_MODE,
  SIZE_REFERENCE
};
}  // namespace reference_description

namespace interaction_mode {
/**
 * @brief Interaction control mode.
 *        1 Interaction on (object manipulation),
 *        0 Interaction off (free flight).
 */
enum InteractionMode { FREE_FLIGHT = 0, INTERACTION };
}  // namespace interaction_mode

/**
 * @brief Names of the different frames used.
 */
struct Frames {
  std::string omav;         //!< Frame positioned at center of omav
  std::string tip;          //!< Frame positioned at the end of the rod
  std::string hook;         //!< Frame positioned in the middle of the hook
  std::string handle_link;  //!< Frame positioned on the handle
  std::string handle_ref;   //!< Frame positioned inside the handle (open space)
  Frames()
      : omav("omav"),
        tip("tip"),
        hook("hook"),
        handle_link("handle_link"),
        handle_ref("handle_ref") {}
};

namespace cost_description {
/**
 * @brief Full cost vector with entries for the different used costs.
 */
enum CostDescription {
  FLOOR_COST = 0,
  POSE_COST,
  OBJECT_COST,
  HANDLE_HOOK_COST,
  TIP_VELOCITY_COST,
  TORQUE_COST,
  EFFICIENCY_COST,
  VELOCITY_COST,
  FORCE_COST,
  LEAVING_FIELD_COST,
  CONTACT_COST,
  UNWANTED_CONTACT_COST,
  OBJECT_DISTANCE_COST,
  SIZE_COST_VECTOR
};
}  // namespace cost_description

}  // namespace omav_interaction
#endif  // MPPI_OMAV_INTERACTION_OMAV_INTERACTION_COMMON_H