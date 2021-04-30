#pragma once
#include <mppi/core/config.h>
#include <mppi/core/rollout.h>
#include <mppi/core/typedefs.h>
#include <mppi/utils/timer.h>

#include <GLFW/glfw3.h>

struct ScrollingBuffer {
  int MaxSize;
  int Offset;
  std::vector<double> x;
  std::vector<double> y;
  ScrollingBuffer(int max_size = 1000);
  void AddPoint(double x, double y);
  void Erase();
  int size();
  bool empty();
  double back_x();
  double back_y();
};

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
  void reset_optimization_time(const double& dt);

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

  // visualization data
  ScrollingBuffer frequency_data;

};
}  // namespace mppi_tools
