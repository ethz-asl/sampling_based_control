//
// Created by giuseppe on 13.09.21.
//

#pragma once
#include "mppi/core/typedefs.h"

namespace mppi {

/// Generic interface class for filtering the input
class Filter {
 public:
  Filter() = default;
  ~Filter() = default;

  virtual void reset(const mppi::observation_t& x, const double t) = 0;
  virtual void apply(const mppi::observation_t& x, mppi::input_t& u,
                     const double t) = 0;
};

}  // namespace mppi
