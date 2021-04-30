#pragma once
#include <mppi/core/rollout.h>
#include <mppi/core/config.h>
#include <mppi/core/typedefs.h>

#include <GLFW/glfw3.h>

namespace mppi_tools {

class ControlGui {
 public:
  ControlGui() = default;
  ~ControlGui();

  bool init();
  bool render();
  void update();
  void draw();
  bool close();

  void window_resize(int width, int height);

  void reset_config(const mppi::config_t& config);
  void reset_averaged_policy(const mppi::input_array_t& u);
  void reset_policy(const mppi::input_array_t& u);
  void reset_rollouts(const std::vector<mppi::Rollout>& rollouts);
  void reset_weights(const std::vector<double>& weights);

  bool should_pause() const { return pause; }

 private:
  bool setup_glfw();

 private:
  GLFWwindow* window_ptr_;

  // control
  bool pause = false;

  // mppi data
  mppi::Config config_;
  std::vector<mppi::Rollout> rollouts_;
  mppi::input_array_t u_;
  mppi::input_array_t u_avg_;
  std::vector<double> weights_;
};
}  // namespace mppi_tools
