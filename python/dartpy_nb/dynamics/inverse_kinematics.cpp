#include "dynamics/inverse_kinematics.hpp"

#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defInverseKinematics(nb::module_& m)
{
  using IK = dart::dynamics::InverseKinematics;
  using ErrorMethod = IK::ErrorMethod;

  nb::class_<ErrorMethod>(m, "InverseKinematicsErrorMethod")
      .def(
          "getMethodName",
          &ErrorMethod::getMethodName,
          nb::rv_policy::reference_internal)
      .def(
          "getBounds",
          [](const ErrorMethod& self) -> ErrorMethod::Bounds {
            return self.getBounds();
          })
      .def(
          "setBounds",
          [](ErrorMethod& self,
             const Eigen::Vector6d& lower,
             const Eigen::Vector6d& upper) { self.setBounds(lower, upper); },
          nb::arg("lower"),
          nb::arg("upper"));

  nb::class_<IK>(m, "InverseKinematics")
      .def("isActive", &IK::isActive)
      .def("setActive", &IK::setActive, nb::arg("active") = true)
      .def(
          "getTarget",
          [](IK& self) -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self.getTarget();
          })
      .def(
          "getErrorMethod",
          [](IK& self) -> ErrorMethod& { return self.getErrorMethod(); },
          nb::rv_policy::reference_internal)
      .def("getSolver", [](IK& self) { return self.getSolver(); })
      .def(
          "setSolver",
          [](IK& self, const std::shared_ptr<dart::optimizer::Solver>& solver) {
            self.setSolver(solver);
          },
          nb::arg("solver"))
      .def("getProblem", [](IK& self) { return self.getProblem(); })
      .def(
          "solveAndApply",
          [](IK& self, bool allowIncompleteResult) {
            return self.solveAndApply(allowIncompleteResult);
          },
          nb::arg("allowIncompleteResult") = true);
}

} // namespace dart::python_nb
