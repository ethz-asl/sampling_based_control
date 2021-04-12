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
  ImGui::ShowDemoWindow(&show_demo_window);
  // ImPlot::ShowDemoWindow(&show_demo_window);

  // This is for resize to cover the full window
  static bool use_work_area = true;
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(use_work_area ? viewport->WorkPos : viewport->Pos);
  ImGui::SetNextWindowSize(use_work_area ? viewport->WorkSize : viewport->Size);
  ImGui::Begin("Test", NULL, ImGuiWindowFlags_AlwaysAutoResize);

  //-------------------------------------------------------------------------//
  if (ImGui::CollapsingHeader("Solver Configuration")) {
    static int clicked = 0;
    if (ImGui::Button("Print")) std::cout << config_ << std::endl;

    ImGui::Checkbox("verbose", &config_.verbose);
    ImGui::Separator();
    ImGui::LabelText("label", "Value");

    {
      ImGui::InputScalarN("input variance", ImGuiDataType_Float,
                          config_.input_variance.data(),
                          config_.input_variance.size());

      // Using the _simplified_ one-liner Combo() api here
      // See "Combo" section for examples of how to use the more flexible
      // BeginCombo()/EndCombo() api.
      const char* items[] = {"AAAA",    "BBBB", "CCCC",   "DDDD",
                             "EEEE",    "FFFF", "GGGG",   "HHHH",
                             "IIIIIII", "JJJJ", "KKKKKKK"};
      static int item_current = 0;
      ImGui::Combo("combo", &item_current, items, IM_ARRAYSIZE(items));
      ImGui::SameLine();
      HelpMarker(
          "Using the simplified one-liner Combo API here.\nRefer to the "
          "\"Combo\" section below for an explanation of how to use the more "
          "flexible and general BeginCombo/EndCombo API.");
    }

    {
      // To wire InputText() with std::string or any other custom string type,
      // see the "Text Input > Resize Callback" section of this demo, and the
      // misc/cpp/imgui_stdlib.h file.
      static char str0[128] = "Hello, world!";
      ImGui::InputText("input text", str0, IM_ARRAYSIZE(str0));
      ImGui::SameLine();
      HelpMarker(
          "USER:\n"
          "Hold SHIFT or use mouse to select text.\n"
          "CTRL+Left/Right to word jump.\n"
          "CTRL+A or double-click to select all.\n"
          "CTRL+X,CTRL+C,CTRL+V clipboard.\n"
          "CTRL+Z,CTRL+Y undo/redo.\n"
          "ESCAPE to revert.\n\n"
          "PROGRAMMER:\n"
          "You can use the ImGuiInputTextFlags_CallbackResize facility if you "
          "need to wire InputText() "
          "to a dynamic string type. See misc/cpp/imgui_stdlib.h for an "
          "example (this is not demonstrated "
          "in imgui_demo.cpp).");

      static char str1[128] = "";
      ImGui::InputTextWithHint("input text (w/ hint)", "enter text here", str1,
                               IM_ARRAYSIZE(str1));

      static int i0 = 123;
      ImGui::InputInt("input int", &i0);
      ImGui::SameLine();
      HelpMarker(
          "You can apply arithmetic operators +,*,/ on numerical values.\n"
          "  e.g. [ 100 ], input \'*2\', result becomes [ 200 ]\n"
          "Use +- to subtract.");

      static float f0 = 0.001f;
      ImGui::InputFloat("input float", &f0, 0.01f, 1.0f, "%.3f");

      static double d0 = 999999.00000001;
      ImGui::InputDouble("input double", &d0, 0.01f, 1.0f, "%.8f");

      static float f1 = 1.e10f;
      ImGui::InputFloat("input scientific", &f1, 0.0f, 0.0f, "%e");
      ImGui::SameLine();
      HelpMarker(
          "You can input value using the scientific notation,\n"
          "  e.g. \"1e+8\" becomes \"100000000\".");

      static float vec4a[4] = {0.10f, 0.20f, 0.30f, 0.44f};
      ImGui::InputFloat3("input float3", vec4a);
    }

    {
      static int i1 = 50, i2 = 42;
      ImGui::DragInt("drag int", &i1, 1);
      ImGui::SameLine();
      HelpMarker(
          "Click and drag to edit value.\n"
          "Hold SHIFT/ALT for faster/slower edit.\n"
          "Double-click or CTRL+click to input value.");

      ImGui::DragInt("drag int 0..100", &i2, 1, 0, 100, "%d%%",
                     ImGuiSliderFlags_AlwaysClamp);

      static float f1 = 1.00f, f2 = 0.0067f;
      ImGui::DragFloat("drag float", &f1, 0.005f);
      ImGui::DragFloat("drag small float", &f2, 0.0001f, 0.0f, 0.0f,
                       "%.06f ns");
    }

    {
      static int i1 = 0;
      ImGui::SliderInt("slider int", &i1, -1, 3);
      ImGui::SameLine();
      HelpMarker("CTRL+click to input value.");

      static float f1 = 0.123f, f2 = 0.0f;
      ImGui::SliderFloat("slider float", &f1, 0.0f, 1.0f, "ratio = %.3f");
      ImGui::SliderFloat("slider float (log)", &f2, -10.0f, 10.0f, "%.4f",
                         ImGuiSliderFlags_Logarithmic);

      static float angle = 0.0f;
      ImGui::SliderAngle("slider angle", &angle);

      // Using the format string to display a name instead of an integer.
      // Here we completely omit '%d' from the format string, so it'll only
      // display a name. This technique can also be used with DragInt().
      enum Element {
        Element_Fire,
        Element_Earth,
        Element_Air,
        Element_Water,
        Element_COUNT
      };
      static int elem = Element_Fire;
      const char* elems_names[Element_COUNT] = {"Fire", "Earth", "Air",
                                                "Water"};
      const char* elem_name =
          (elem >= 0 && elem < Element_COUNT) ? elems_names[elem] : "Unknown";
      ImGui::SliderInt("slider enum", &elem, 0, Element_COUNT - 1, elem_name);
      ImGui::SameLine();
      HelpMarker(
          "Using the format string parameter to display a name instead of the "
          "underlying integer.");
    }

    {
      static float col1[3] = {1.0f, 0.0f, 0.2f};
      static float col2[4] = {0.4f, 0.7f, 0.0f, 0.5f};
      ImGui::ColorEdit3("color 1", col1);
      ImGui::SameLine();
      HelpMarker(
          "Click on the color square to open a color picker.\n"
          "Click and hold to use drag and drop.\n"
          "Right-click on the color square to show options.\n"
          "CTRL+click on individual component to input value.\n");

      ImGui::ColorEdit4("color 2", col2);
    }

    {
      // Using the _simplified_ one-liner ListBox() api here
      // See "List boxes" section for examples of how to use the more flexible
      // BeginListBox()/EndListBox() api.
      const char* items[] = {"Apple",     "Banana",     "Cherry",
                             "Kiwi",      "Mango",      "Orange",
                             "Pineapple", "Strawberry", "Watermelon"};
      static int item_current = 1;
      ImGui::ListBox("listbox", &item_current, items, IM_ARRAYSIZE(items), 4);
      ImGui::SameLine();
      HelpMarker(
          "Using the simplified one-liner ListBox API here.\nRefer to the "
          "\"List boxes\" section below for an explanation of how to use the "
          "more flexible and general BeginListBox/EndListBox API.");
    }
  }

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

void VisualDebugger::reset_config(const SolverConfig& config) {
  config_ = config;
}

void VisualDebugger::reset_policy(const input_t& u) {}

void VisualDebugger::reset_filtered_policy(const input_t& u) {}

void VisualDebugger::reset_rollouts(const std::vector<Rollout>& rollouts) {}

void VisualDebugger::reset_weights(const Eigen::ArrayXd& weights) {
  weights_ = weights;
}

}  // namespace mppi_tools
