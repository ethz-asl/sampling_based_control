/*!
 * @file     rederer.h
 * @author   Giuseppe Rizzi
 * @date     09.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <vector>
#include "mppi/controller/rollout.h"

namespace mppi {
class Renderer {
 public:
  Renderer() = default;
  ~Renderer() = default;

  virtual void render(const std::vector<Rollout>&) = 0;
};

}  // namespace mppi
