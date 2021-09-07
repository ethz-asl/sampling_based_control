//
// Created by giuseppe on 25.05.21.
//

#include "mppi/policies/spline_policy.h"
#include "mppi/policies/gaussian_policy.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>

using namespace mppi;
namespace py = pybind11;

PYBIND11_MODULE(pymppi, m) {
  m.def("B", &B, "A function which return a basis function");

  py::class_<mppi::BSplinePolicyConfig>(m, "BSplinePolicyConfig")
      .def(py::init<>())
      .def_readwrite("degree", &BSplinePolicyConfig::degree)
      .def_readwrite("horizon", &BSplinePolicyConfig::horizon)
      .def_readwrite("cp_dt", &BSplinePolicyConfig::cp_dt)
      .def_readwrite("dt", &BSplinePolicyConfig::dt)
      .def_readwrite("sigma", &BSplinePolicyConfig::sigma)
      .def_readwrite("samples", &BSplinePolicyConfig::samples)
      .def_readwrite("verbose", &BSplinePolicyConfig::verbose)
      .def_readwrite("max_value", &BSplinePolicyConfig::max_value)
      .def_readwrite("min_value", &BSplinePolicyConfig::min_value);

  py::class_<mppi::RecedingHorizonSpline>(m, "RecedingHorizonSpline")
      .def(py::init<const BSplinePolicyConfig&>())
      .def("compute_nominal", &RecedingHorizonSpline::compute_nominal)
      .def("update_samples", &RecedingHorizonSpline::update_samples)
      .def("get_samples", &RecedingHorizonSpline::get_samples)
      .def("get_gradients", &RecedingHorizonSpline::get_gradients)
      .def("get_gradients_matrix", &RecedingHorizonSpline::get_gradients_matrix)
      .def("update", &RecedingHorizonSpline::update, py::call_guard<py::scoped_ostream_redirect,
          py::scoped_estream_redirect>())
      .def("get_sample_control_polygon", &RecedingHorizonSpline::get_sample_control_polygon)
      .def("get_time", &RecedingHorizonSpline::get_time)
      .def("control_polygon_t", &RecedingHorizonSpline::get_control_polygon_t)
      .def("control_polygon_y", &RecedingHorizonSpline::get_control_polygon_y)
      .def("shift", &RecedingHorizonSpline::shift);

  py::class_<mppi::Config>(m, "Config")
      .def(py::init<>())
      .def_readwrite("horizon", &Config::horizon)
      .def_readwrite("step_size", &Config::step_size)
      .def_readwrite("variance", &Config::input_variance)
      .def_readwrite("filtering", &Config::filtering)
      .def_readwrite("filters_window", &Config::filters_window)
      .def_readwrite("filters_order", &Config::filters_order)
      .def_readwrite("u_max", &Config::u_max)
      .def_readwrite("u_min", &Config::u_min)
      .def_readwrite("samples", &Config::rollouts);

  py::class_<mppi::GaussianPolicy>(m, "GaussianPolicy")
      .def(py::init<int, const mppi::Config&>())
      .def("update_samples", &GaussianPolicy::update_samples)
      .def("update", &GaussianPolicy::update)
      .def("shift", &GaussianPolicy::shift)
      .def("get", &GaussianPolicy::nominal)
      .def("get_sample", &GaussianPolicy::sample)
      .def("get_time", &GaussianPolicy::get_time)
      .def("update_delay", &GaussianPolicy::update_delay);

  py::class_<mppi::SplinePolicy>(m, "SplinePolicy")
      .def(py::init<int, const mppi::Config&>())
      .def("update_samples", &SplinePolicy::update_samples)
      .def("update", &SplinePolicy::update)
      .def("shift", &SplinePolicy::shift)
      .def("get", &SplinePolicy::nominal)
      .def("get_sample", &SplinePolicy::sample)
      .def("get_time", &SplinePolicy::get_time);
}
