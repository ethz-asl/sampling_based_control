#pragma once
#include <mppi/controller/rollout.h>
#include <mppi/solver_config.h>
#include <mppi/typedefs.h>

#include <GLFW/glfw3.h>

namespace mppi_tools {

class VisualDebugger {
 public:
  VisualDebugger() = default;
  ~VisualDebugger();

  bool init();
  bool render();
  void update();
  void draw();
  bool close();

  void window_resize(int width, int height);

  void create_config(const mppi::SolverConfig* config);
  void reset_policy(const mppi::input_t& u);
  void reset_filtered_policy(const mppi::input_t& u);
  void reset_rollouts(const std::vector<mppi::Rollout>& rollouts);
  void reset_weights(const Eigen::ArrayXd& weights);

 private:
  bool setup_glfw();

 private:
  GLFWwindow* window_ptr_;

  // mppi data
  mppi::SolverConfig config_;
  std::vector<mppi::Rollout> rollouts_;
  mppi::input_t u_;
  mppi::input_t u_filt_;
  Eigen::ArrayXd weights_;
};
}  // namespace mppi_tools
