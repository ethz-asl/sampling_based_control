/*
 * Created by Giulio Schiavi
 * 4 NOV 2021
 * Generates a shared library file at catkin_ws/devel/.private/mppi_manipulation/lib/pymppi_manipulation.cpython-38-x86_64-linux-gnu.so
 */


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>

#include "mppi_manipulation/controller_interface_no_ros.h"

using namespace manipulation;
namespace py = pybind11;

PYBIND11_MODULE(pymppi_manipulation, m) {
  m.doc() = "python library for mppi manipulation controller"; // optional module docstring
  py::class_<PandaControllerInterfaceNoRos>(m, "PandaControllerInterface")
      .def(py::init<const std::string &>())
      .def("init", &PandaControllerInterfaceNoRos::init)
      .def("set_observation", &PandaControllerInterfaceNoRos::set_observation)
      .def("update_policy", &PandaControllerInterfaceNoRos::update_policy)
      .def("get_input", &PandaControllerInterfaceNoRos::get_input)
      .def("set_reference", &PandaControllerInterfaceNoRos::set_reference);
}
