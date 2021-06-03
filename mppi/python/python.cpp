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

PYBIND11_MODULE(pysplines, m) {
  m.def("B", &B, "A function which return a basis function");

  py::class_<mppi::BSplinePolicyConfig>(m, "BSplinePolicyConfig")
      .def(py::init<>())
      .def_readwrite("degree", &BSplinePolicyConfig::degree)
      .def_readwrite("horizon", &BSplinePolicyConfig::horizon)
      .def_readwrite("cp_dt", &BSplinePolicyConfig::cp_dt)
      .def_readwrite("dt", &BSplinePolicyConfig::dt)
      .def_readwrite("sigma", &BSplinePolicyConfig::sigma)
      .def_readwrite("samples", &BSplinePolicyConfig::samples)
      .def_readwrite("verbose", &BSplinePolicyConfig::verbose);

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

  py::class_<mppi::GaussianPolicy>(m, "GaussianPolicy")
      .def(py::init<int, int, double, double, std::vector<int>, std::vector<unsigned int>, const Eigen::VectorXd&>())
      .def("update_samples", &GaussianPolicy::update_samples)
      .def("update", &GaussianPolicy::update)
      .def("shift", &GaussianPolicy::shift)
      .def("get", &GaussianPolicy::get)
      .def("get_sample", &GaussianPolicy::get_sample)
      .def("get_time", &GaussianPolicy::get_time);
}
//
//GaussianPolicy(int nu, int ns, double dt, double horizon, const Eigen::VectorXd& sigma);
//
//Eigen::VectorXd operator()(double t) override;
//
//Eigen::VectorXd operator()(double t, int k) override;
//
//void update_samples(const Eigen::VectorXd& weights, const int keep) override;
//
//void update(const Eigen::VectorXd& weights, const double step_size) override;
//
//void shift(const double t) override;