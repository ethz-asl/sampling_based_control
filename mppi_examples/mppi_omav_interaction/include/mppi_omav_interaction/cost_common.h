#pragma once
#include <string>

namespace omav_interaction {

struct Frames {
  std::string omav;
  std::string hook;
  std::string tip;
  std::string handle_link;
  std::string handle_ref;
  Frames()
      : omav("omav"),
        hook("hook"),
        tip("tip"),
        handle_link("handle_link"),
        handle_ref("handle_ref") {}
};

namespace CostIdx {
  enum IdxEnum {
    floor = 0,
    pose = 1,
    object = 2,
    handle_hook = 3,
    tip_velocity = 4,
    torque = 5,
    efficiency = 6,
    velocity = 7,
    force = 8,
    leaving_field = 9,
    contact = 10
  };
}
constexpr size_t kN_costs = 11;
const std::string kCost_descr[kN_costs] = {
    "floor",        "pose",          "object",     "handle_hook",
    "tip_velocity", "torque",        "efficiency", "velocity",
    "force",        "leaving_field", "contact"};
}  // namespace omav_interaction