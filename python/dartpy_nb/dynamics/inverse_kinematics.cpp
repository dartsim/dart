#include "dynamics/inverse_kinematics.hpp"

#include "dart/common/Diagnostics.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/optimizer/Solver.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

DART_SUPPRESS_DEPRECATED_BEGIN

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
          "setLinearBounds",
          [](ErrorMethod& self,
             const Eigen::Vector3d& lower,
             const Eigen::Vector3d& upper) { self.setLinearBounds(lower, upper); },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setAngularBounds",
          [](ErrorMethod& self,
             const Eigen::Vector3d& lower,
             const Eigen::Vector3d& upper) { self.setAngularBounds(lower, upper); },
          nb::arg("lower"),
          nb::arg("upper"))
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
      .def("useWholeBody", &IK::useWholeBody)
      .def(
          "setTarget",
          [](IK& self,
             const std::shared_ptr<dart::dynamics::SimpleFrame>& target) {
            self.setTarget(target);
          },
          nb::arg("target"),
          nb::keep_alive<1, 2>())
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
          nb::arg("solver"),
          nb::keep_alive<1, 2>())
      .def("getProblem", [](IK& self) { return self.getProblem(); })
      .def(
          "solveAndApply",
          [](IK& self, bool allowIncompleteResult) {
            return self.solveAndApply(allowIncompleteResult);
          },
          nb::arg("allowIncompleteResult") = true);

  m.def(
      "createSimpleFrame",
      [](dart::dynamics::Frame* parent,
         const std::string& name,
         const Eigen::Isometry3d& tf) {
        auto* frame = new dart::dynamics::SimpleFrame(parent, name, tf);
        return std::shared_ptr<dart::dynamics::SimpleFrame>(frame);
      },
      nb::arg("parent"),
      nb::arg("name"),
      nb::arg("transform"),
      nb::rv_policy::take_ownership);
}

DART_SUPPRESS_DEPRECATED_END

} // namespace dart::python_nb
