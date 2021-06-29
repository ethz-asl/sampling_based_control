#include "mppi_tools/control_gui.hpp"
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"

static void error_callback(int error, const char* description) {
  fputs(description, stderr);
}

static void resize_callback(GLFWwindow* window, int width, int height) {
  auto g =
      static_cast<mppi_tools::ControlGui*>(glfwGetWindowUserPointer(window));
  g->window_resize(width, height);
}

static void HelpMarker(const char* desc) {
  ImGui::TextDisabled("(?)");
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

struct RolloutData {
  const mppi::Rollout* rollout;
  int input_selection = 0;
};

ImPlotPoint RolloutInputPoint(void* data, int idx) {
  auto* d = (RolloutData*)data;
  double x = d->rollout->tt[idx];
  double y = d->rollout->uu[idx](d->input_selection);
  return ImPlotPoint(x, y);
}

using namespace mppi;

namespace mppi_tools {

ControlGui::~ControlGui() {
  // Cleanup
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_ptr_);
  glfwTerminate();
}

bool ControlGui::setup_glfw() {
  // setup the callback
  glfwSetErrorCallback(error_callback);

  // check the glfw initialized properly
  if (!glfwInit()) {
    return false;
  }

  // Initialise the new window
  window_ptr_ = glfwCreateWindow(640, 480, "Control GUI", NULL, NULL);

  // Resize callback
  glfwSetWindowUserPointer(window_ptr_, this);
  glfwSetWindowSizeCallback(window_ptr_, resize_callback);

  // check to see if the window initialized properly
  if (!window_ptr_) {
    glfwTerminate();
    return false;
  }

  // make the new window the current context
  glfwMakeContextCurrent(window_ptr_);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable
  // Keyboard Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; //
  // Enable Gamepad Controls

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window_ptr_, true);
  ImGui_ImplOpenGL2_Init();

  return true;
}

bool ControlGui::init() { return setup_glfw(); }

bool ControlGui::render() {
  if (!glfwWindowShouldClose(window_ptr_)) {
    glfwPollEvents();

    // call the update and draw function
    update();
    draw();
    return true;
  } else {
    return false;
  }
}

void ControlGui::update() {}

void ControlGui::draw() {
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  bool show_demo_window = true;
  ImGui::ShowDemoWindow(&show_demo_window);
  // ImPlot::ShowDemoWindow(&show_demo_window);

  // This is for resize to cover the full window
  static bool use_work_area = true;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(use_work_area ? viewport->WorkPos : viewport->Pos);
  ImGui::SetNextWindowSize(use_work_area ? viewport->WorkSize : viewport->Size);

  // Window flags
  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_MenuBar;
  window_flags |= ImGuiWindowFlags_AlwaysAutoResize;
  ImGui::Begin("Test", NULL, window_flags);

  //-------------------------------------------------------------------------//
  //                       Menu
  //-------------------------------------------------------------------------//
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      if (ImGui::BeginMenu("Style")) {
        if (ImGui::MenuItem("Dark")) ImGui::StyleColorsDark();
        if (ImGui::MenuItem("Light")) ImGui::StyleColorsLight();
        if (ImGui::MenuItem("Classic")) ImGui::StyleColorsClassic();
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }

  //-------------------------------------------------------------------------//
  //                       Common
  //-------------------------------------------------------------------------//
  ImGui::Checkbox("Pause", &pause);
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  ImGui::Text("Step:");
  ImGui::SameLine();

  step_simulation_ = ImGui::Button("Step Simulation");
  ImGui::SameLine();

  step_controller_ = ImGui::Button("Step Controller");
  ImGui::SameLine();

  step_all_ = ImGui::Button("Step All");
  ImGui::SameLine();

  // Arrow buttons with Repeater
  static int counter = 0;
  float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
  ImGui::PushButtonRepeat(true);
  if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
    counter--;
  }
  ImGui::SameLine(0.0f, spacing);
  if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
    counter++;
  }
  ImGui::PopButtonRepeat();
  ImGui::SameLine();
  ImGui::Text("%d", counter);

  ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
  if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
    //-------------------------------------------------------------------------//
    //                       Configuration
    //-------------------------------------------------------------------------//
    if (ImGui::BeginTabItem("Configuration")) {
      static int clicked = 0;
      if (ImGui::Button("Print")) std::cout << config_ << std::endl;
      ImGui::Checkbox("verbose", &config_.verbose);
      ImGui::EndTabItem();
    }

    //-------------------------------------------------------------------------//
    //                       Inputs info
    //-------------------------------------------------------------------------//
    if (ImGui::BeginTabItem("Input")) {
      if (!u_.empty()) {
        static float history = 10.0f;
        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");
        float t = u_[0].back().x;
        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-1.0, 1.0, ImGuiCond_Once);
        if (ImPlot::BeginPlot("##ID", "t", "u")) {
          for (int i = 0; i < u_.size(); i++) {
            char label[1];
            label[0] = i + '0';
            ImPlot::PlotLine(label, &u_[i].data[0].x, &u_[i].data[0].y,
                             u_[i].data.size(), u_[i].offset,
                             2 * sizeof(float));
          }
          ImPlot::EndPlot();
        }
      }
      ImGui::EndTabItem();
    }

    //-------------------------------------------------------------------------//
    //                       Rollouts info
    //-------------------------------------------------------------------------//
    if (ImGui::BeginTabItem("Rollouts")) {
      static bool show_all = false;
      static bool show_filtered = false;
      static int nr_rollouts = (int)rollouts_.size();
      static int rollout_idx = 0;
      static int input_idx = 0;
      static int nr_inputs = (int)rollouts_[0].uu[0].size();

      ImGui::AlignTextToFramePadding();
      ImGui::SliderInt("rollout index", &rollout_idx, 0, nr_rollouts - 1);
      ImGui::SameLine();
      ImGui::Checkbox("show all", &show_all);
      ImGui::SameLine();
      ImGui::SameLine();
      ImGui::Checkbox("show filtered", &show_filtered);
      ImGui::EndTabItem();
      ImGui::SameLine();

      static bool first_run = true;
      static bool* selected_input = new bool[nr_inputs];
      static char** input_names = new char*[nr_inputs];

      if (first_run) {
        for (size_t i = 0; i < nr_inputs; i++) {
          input_names[i] = new char[10];
          strcpy(input_names[i], "input_");
          input_names[i][6] = i + '0';  // convert integer to char
          selected_input[i] = false;
          first_run = false;
        }
      }

      ImGui::SameLine();
      if (ImGui::Button("select input")) ImGui::OpenPopup("selection_popup");
      if (ImGui::BeginPopup("selection_popup")) {
        for (size_t i = 0; i < nr_inputs; i++) {
          ImGui::MenuItem(input_names[i], "", &selected_input[i]);
        }
        ImGui::EndPopup();
      }

      for (int i = 0; i < nr_inputs; i++) {
        ImPlot::SetNextPlotLimitsX(rollouts_[0].tt[0], rollouts_[0].tt.back(),
                                   ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-1.0, 1.0, ImGuiCond_Once);
        if (selected_input[i]) {
          if (ImPlot::BeginPlot(input_names[i], "x", "f(x)")) {
            if (show_all) {
              for (const auto& roll : rollouts_) {
                RolloutData data;
                data.rollout = &roll;
                data.input_selection = i;
                ImPlot::PlotLineG("##Legend", RolloutInputPoint, &data,
                                  (int)roll.tt.size());
              }
            } else {
              RolloutData data;
              data.rollout = &rollouts_[rollout_idx];
              data.input_selection = i;
              ImPlot::PlotLineG("u", RolloutInputPoint, &data,
                                (int)data.rollout->tt.size());
            }

            if (show_filtered) {
              std::vector<double> u;
              std::vector<double> t = rollouts_[0].tt;
              for (int j = 0; j < uu_.size(); j++) u.push_back(uu_[j][i]);
              ImPlot::PlotLine("u_filtered", t.data(), u.data(), (int)t.size());
            }
            ImPlot::EndPlot();
          }
        }
      }
    }

    //-------------------------------------------------------------------------//
    //                       Reference
    //-------------------------------------------------------------------------//
    if (ImGui::BeginTabItem("Reference")) {
      ImGui::EndTabItem();
    }

    //-------------------------------------------------------------------------//
    //                       Statistics
    //-------------------------------------------------------------------------//
    if (ImGui::BeginTabItem("Statistics")) {
      if (ImGui::CollapsingHeader("Rollouts weights")) {
        static std::vector<double> weights_ranges = {0.01, 0.05, 0.1,
                                                     0.2,  0.5,  1.0};
        static double ymax_prev = 0.01;
        static double ymax_counter = 0.0;
        if (weights_.size() == 0) {
          ImGui::Text("Weights are not set. Cannot display weights.");
        } else {
          std::vector<double> ridx;
          for (int i = 0; i < (int)weights_.size(); i++) ridx.push_back(i);

          // dynamic y range
          double wmax = *std::max_element(weights_.begin(), weights_.end());
          double ymax = *std::lower_bound(weights_ranges.begin(),
                                          weights_ranges.end(), wmax);
          // a lower value must stay long to push down a higher threshold
          if (ymax > ymax_prev) {
            ImPlot::SetNextPlotLimitsY(-0.01, ymax, ImGuiCond_Always);
            ymax_counter = 0;
            ymax_prev = ymax;
          } else if (ymax == ymax_prev and ymax_counter > 100) {
            ImPlot::SetNextPlotLimitsY(-0.01, ymax, ImGuiCond_Always);
          } else if (ymax == ymax_prev) {
            ymax_counter++;
          } else {
            ymax_prev = ymax;
            ymax_counter = 0;
          }
          ImPlot::SetNextPlotLimitsX(0, (int)weights_.size(), ImGuiCond_Once);
          if (ImPlot::BeginPlot("rollouts weights", "rollout index", "weight",
                                ImVec2(-1, 0), ImPlotFlags_NoLegend,
                                ImPlotAxisFlags_Lock)) {
            ImPlot::PlotLine("weights", ridx.data(), weights_.data(),
                             (int)weights_.size());
            ImPlot::EndPlot();
          }
        }
      }

      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();

  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(window_ptr_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.45, 0.56, 0.55, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
  glfwMakeContextCurrent(window_ptr_);
  glfwSwapBuffers(window_ptr_);
}

void ControlGui::window_resize(int width, int height) {
  if (width == 0 || height == 0) return;

  ImVec2 size(width, height);
  ImGui::SetNextWindowSize(size);
}

bool ControlGui::close() { return true; }

void ControlGui::reset_config(const mppi::config_t& config) {
  config_ = config;
}

void ControlGui::reset_averaged_policy(const input_array_t& u) { uu_avg_ = u; }

void ControlGui::reset_input(const mppi::input_t& u, const double t) {
  static bool first_reset = true;
  if (first_reset) {
    u_.resize(u.size(), scrolling_buffer_t(1000));
    first_reset = false;
  }
  for (int i = 0; i < u.size(); i++) {
    u_[i].add_point(t, u[i]);
  }
}

void ControlGui::reset_policy(const input_array_t& u) { uu_ = u; }

void ControlGui::reset_rollouts(const std::vector<Rollout>& rollouts) {
  rollouts_ = rollouts;
}

void ControlGui::reset_weights(const std::vector<double>& weights) {
  weights_ = weights;
}

}  // namespace mppi_tools
