#pragma once
#include <functional>

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

template <typename T>
struct param_options_t {
  using cb = std::function<bool(const T&)>;
  T value;
  T default_value;
  T lower;
  T upper;
  std::function<bool(const T&)> callback;
};

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

  bool register_param(const std::string& param_name,
                      param_options_t<float>& param_opt);

  void reset_config(const mppi::config_t& config);
  void reset_averaged_policy(const mppi::input_array_t& u);
  void reset_policy(const mppi::input_array_t& u);
  void reset_rollouts(const std::vector<mppi::Rollout>& rollouts);
  void reset_weights(const std::vector<double>& weights);
  void reset_optimization_time(const double& dt);

  inline bool should_pause() const { return pause_; }
  inline bool should_start() const { return start_; }

 private:
  bool setup_glfw();

 private:
  GLFWwindow* window_ptr_;

  bool start_ = false;
  bool pause_ = false;

  // mppi data
  mppi::Config config_;
  std::vector<mppi::Rollout> rollouts_;
  mppi::input_array_t u_;
  mppi::input_array_t u_avg_;
  std::vector<double> weights_;

  // visualization data
  ScrollingBuffer frequency_data;

  // params
  std::map<std::string, param_options_t<float>> params_list_;
};
}  // namespace mppi_tools
