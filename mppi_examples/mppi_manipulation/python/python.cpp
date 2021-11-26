/*
 * Created by Giulio Schiavi
 * 4 NOV 2021
 * Generates a shared library file at catkin_ws/devel/.private/mppi_manipulation/lib/pymppi_manipulation.cpython-38-x86_64-linux-gnu.so
 */


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "mppi_manipulation/controller_interface_no_ros.h"
#include "mppi_manipulation/dynamics.h"

using namespace manipulation;
namespace py = pybind11;

PYBIND11_MODULE(pymppi_manipulation, m) {
  m.doc() = "python library for mppi manipulation controller"; // optional module docstring
  py::class_<PandaControllerInterfaceNoRos>(m, "PandaControllerInterface")
      .def(py::init<const std::string &>())
      .def("init", &PandaControllerInterfaceNoRos::init, py::return_value_policy::copy)
      .def("set_observation", &PandaControllerInterfaceNoRos::set_observation, py::return_value_policy::copy)
      .def("update_policy", &PandaControllerInterfaceNoRos::update_policy, py::return_value_policy::copy)
      .def("get_input", &PandaControllerInterfaceNoRos::get_input, py::return_value_policy::copy)
      .def("get_rollout_cost", &PandaControllerInterfaceNoRos::get_rollout_cost, py::return_value_policy::copy)
      .def("get_stage_cost", &PandaControllerInterfaceNoRos::get_stage_cost, py::return_value_policy::copy)
      .def("get_cost_map", &PandaControllerInterfaceNoRos::get_cost_map, py::return_value_policy::copy)
      .def("set_reference", &PandaControllerInterfaceNoRos::set_reference, py::return_value_policy::copy);
  py::class_<PandaRaisimDynamics>(m, "PandaRaisimDynamics")
      .def(py::init<const std::string &>())
      .def("step", &PandaRaisimDynamics::step, py::return_value_policy::copy)
      .def("get_dt", &PandaRaisimDynamics::get_dt, py::return_value_policy::copy)
      .def("get_state", &PandaRaisimDynamics::get_state, py::return_value_policy::copy);
}
