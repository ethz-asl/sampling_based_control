#pragma once

#include <stdexcept>

namespace mppi {

class DynamicsDivergedError : public std::runtime_error {
 public:
  DynamicsDivergedError()
      : std::runtime_error("Something went wrong ... dynamics diverged?"){};
};

}  // namespace mppi