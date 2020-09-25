/*!
 * @file     gui.h
 * @author   Giuseppe Rizzi
 * @date     15.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Core>

#include <GLFW/glfw3.h>


static void glfw_error_callback(int error, const char* description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
};

namespace mppi_gui {

class MppiGui {
 public:
  MppiGui(){
    setUp();
  }
  ~MppiGui() {
    tearDown();
  }

 private:
  std::vector<double> weights_x_;
  std::vector<double> weights_y_;

  std::vector<double> costs_x_;
  std::vector<double> costs_y_;

  GLFWwindow *window_;

 private:
  bool setUp();
  bool shouldClose();
  bool tearDown();

 public:
  bool render();
  void set_data(const Eigen::ArrayXd& weights, const Eigen::ArrayXd& costs);
};


}