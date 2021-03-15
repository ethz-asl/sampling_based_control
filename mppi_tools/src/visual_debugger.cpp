#include "mppi_tools/visual_debugger.hpp"
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"

static void error_callback(int error, const char* description) {
  fputs(description, stderr);
}

static void resize_callback(GLFWwindow* window, int width, int height) {
  auto g = static_cast<mppi_tools::VisualDebugger*>(
      glfwGetWindowUserPointer(window));
  g->window_resize(width, height);
}

using namespace mppi;

namespace mppi_tools {

VisualDebugger::~VisualDebugger() {
  // Cleanup
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_ptr_);
  glfwTerminate();
}

bool VisualDebugger::setup_glfw() {
  // setup the callback
  glfwSetErrorCallback(error_callback);

  // check the glfw initialized properly
  if (!glfwInit()) {
    return false;
  }

  // Initialise the new window
  window_ptr_ = glfwCreateWindow(640, 480, "MPPI Visual Debugger", NULL, NULL);

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

bool VisualDebugger::init() { return setup_glfw(); }

bool VisualDebugger::render() {
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

void VisualDebugger::update() {}

void VisualDebugger::draw() {
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  bool show_demo_window = true;
  // ImGui::ShowDemoWindow(&show_demo_window);
  // ImPlot::ShowDemoWindow(&show_demo_window);

  // This is for resize to cover the full window
  static bool use_work_area = true;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(use_work_area ? viewport->WorkPos : viewport->Pos);
  ImGui::SetNextWindowSize(use_work_area ? viewport->WorkSize : viewport->Size);
  ImGui::Begin("Test", NULL, ImGuiWindowFlags_AlwaysAutoResize);

  //-------------------------------------------------------------------------//
  if (ImGui::CollapsingHeader("Rollouts Info")) {
    static float xs1[1001], ys1[1001];
    for (int i = 0; i < 1001; ++i) {
      xs1[i] = i * 0.001f;
      ys1[i] = 0.5f + 0.5f * sinf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
    }
    static double xs2[11], ys2[11];
    for (int i = 0; i < 11; ++i) {
      xs2[i] = i * 0.1f;
      ys2[i] = xs2[i] * xs2[i];
    }
    if (ImPlot::BeginPlot("Line Plot", "x", "f(x)")) {
      ImPlot::PlotLine("sin(x)", xs1, ys1, 1001);
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle);
      ImPlot::PlotLine("x^2", xs2, ys2, 11);
      ImPlot::EndPlot();
    }
  }

  //-------------------------------------------------------------------------//
  if (ImGui::CollapsingHeader("Optimization weights")) {
    if (weights_.size() == 0) {
      ImGui::Text("Weights are not set. Cannot display weights.");
    } else {
      ImPlot::SetNextPlotLimitsX(0, weights_.size());
      if (ImPlot::BeginPlot("rollouts weights", "rollout number", "weight",
                            ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
        ImPlot::PlotLine("weights", weights_.data(), weights_.size(),
                         weights_.size());

        ImPlot::EndPlot();
      }
    }
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

void VisualDebugger::window_resize(int width, int height) {
  if (width == 0 || height == 0) return;

  ImVec2 size(width, height);
  ImGui::SetNextWindowSize(size);
}

bool VisualDebugger::close() { return true; }

void VisualDebugger::create_config(const SolverConfig* config) {}

void VisualDebugger::reset_policy(const input_t& u) {}

void VisualDebugger::reset_filtered_policy(const input_t& u) {}

void VisualDebugger::reset_rollouts(const std::vector<Rollout>& rollouts) {}

void VisualDebugger::reset_weights(const Eigen::ArrayXd& weights) {
  weights_ = weights;
}

}  // namespace mppi_tools
