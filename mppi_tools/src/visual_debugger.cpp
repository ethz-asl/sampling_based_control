#include "mppi_tools/visual_debugger.hpp"
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"

static void error_callback(int error, const char* description) {
  fputs(description, stderr);
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
  window_ptr_ = glfwCreateWindow(640, 480, "Chat Room", NULL, NULL);

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
  while (!glfwWindowShouldClose(window_ptr_)) {
    glfwPollEvents();

    // call the update and draw function
    update();
    draw();
  }
  return true;
}

void VisualDebugger::update() {}

void VisualDebugger::draw() {
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  bool show_demo_window = true;
  // ImGui::ShowDemoWindow(&show_demo_window);
  ImPlot::ShowDemoWindow(&show_demo_window);

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

bool VisualDebugger::close() { return true; }

void VisualDebugger::create_config(const SolverConfig* config) {}

void VisualDebugger::reset_policy(const input_t& u) {}

void VisualDebugger::reset_filtered_policy(const input_t& u) {}

void VisualDebugger::reset_rollouts(const std::vector<Rollout>& rollouts) {}

void VisualDebugger::reset_weights(const Eigen::ArrayXd& weights) {}

}  // namespace mppi_tools
