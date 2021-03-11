#include "mppi_tools/visual_debugger.hpp"
#include <iostream>

static void error_callback(int error, const char* description) {
  fputs(description, stderr);
}

using namespace mppi;

namespace mppi_tools {

VisualDebugger::~VisualDebugger() {
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

  // set the background to white
  glClearColor(1, 1, 1, 1);
  return true;
}

bool VisualDebugger::init() { return setup_glfw(); }

bool VisualDebugger::render() {
  while (!glfwWindowShouldClose(window_ptr_)) {
    // Check for the user pressed escape
    if (glfwGetKey(window_ptr_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window_ptr_, true);

    // Getting the window and height for the view port
    int width, height;
    glfwGetFramebufferSize(window_ptr_, &width, &height);
    glViewport(0, 0, width, height);

    // call the update and draw function
    update();
    draw();
  }
  return true;
}

void VisualDebugger::update() {}

void VisualDebugger::draw() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glfwSwapBuffers(window_ptr_);
  glfwPollEvents();
}

bool VisualDebugger::close() { return true; }

void VisualDebugger::create_config(const SolverConfig* config) {}

void VisualDebugger::reset_policy(const input_t& u) {}

void VisualDebugger::reset_filtered_policy(const input_t& u) {}

void VisualDebugger::reset_rollouts(const std::vector<Rollout>& rollouts) {}

void VisualDebugger::reset_weights(const Eigen::ArrayXd& weights) {}

}  // namespace mppi_tools