#include "mppi/gui/gui.h"

using namespace mppi_gui;

void MppiGui::set_data(const Eigen::ArrayXd& weights, const Eigen::ArrayXd& costs){
  weights_x_.clear();
  weights_y_.clear();
  for (size_t i=0; i<weights.size(); i++){
    weights_x_.push_back((double)i);
    weights_y_.push_back(weights(i));
  }
  std::sort(weights_y_.begin(), weights_y_.end(), [](double a, double b){ return a > b; });

  costs_x_.clear();
  costs_y_.clear();
  for (size_t i=0; i<costs.size(); i++){
    costs_x_.push_back((double)i);
    costs_y_.push_back(costs(i));
  }
  std::sort(costs_y_.begin(), costs_y_.end());

}

bool MppiGui::setUp() {

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);

  if (!glfwInit()) {
    return false;
  }
  window_ = glfwCreateWindow(1280, 720, "Dear ImGui GLFW+OpenGL2 example", NULL, NULL);
  if (window_ == NULL) {
    return false;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1); // Enable vsync

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  (void) io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL2_Init();

  weights_x_.resize(0.0, 0);
  weights_y_.resize(0.0, 0);

  costs_x_.resize(0.0, 0);
  costs_y_.resize(0.0, 0);
  return true;
}

bool MppiGui::shouldClose() {
  return glfwWindowShouldClose(window_);
}

bool MppiGui::tearDown() {
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_);
  glfwTerminate();
  return true;
}

bool MppiGui::render() {
  if (!shouldClose()){
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    if (ImGui::CollapsingHeader("Rollouts weights")) {
      ImPlot::SetNextPlotLimitsX(0.0, weights_x_.size(), ImGuiCond_Always);
      if (ImPlot::BeginPlot("weights", "rollouts", "w", ImVec2(-1,0), 0, 0, ImPlotAxisFlags_LogScale)) {
        ImPlot::PlotScatter("w", &weights_x_[0], &weights_y_[0], weights_x_.size());
        ImPlot::EndPlot();
      }
    }
    if (ImGui::CollapsingHeader("Rollouts costs")) {
      ImPlot::SetNextPlotLimitsX(0.0, costs_x_.size(), ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(0.0, costs_y_[costs_x_.size()-1], ImGuiCond_Always);
      if (ImPlot::BeginPlot("cost", "rollouts", "c")) {
        ImPlot::PlotLine("c", &costs_x_[0], &costs_y_[0], costs_x_.size());
        ImPlot::EndPlot();
      }
    }

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glfwMakeContextCurrent(window_);
    glfwSwapBuffers(window_);
  }
}
