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

  bool setup_glfw();

  void create_config(const mppi::SolverConfig* config);
  void reset_policy(const mppi::input_t& u);
  void reset_filtered_policy(const mppi::input_t& u);
  void reset_rollouts(const std::vector<mppi::Rollout>& rollouts);
  void reset_weights(const Eigen::ArrayXd& weights);

 private:
  GLFWwindow* window_ptr_;
};
}  // namespace mppi_tools
