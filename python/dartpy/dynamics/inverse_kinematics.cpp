#include "dynamics/inverse_kinematics.hpp"

#include "common/repr.hpp"
#include "dart/common/diagnostics.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/math/optimization/solver.hpp"

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
  using GradientMethod = IK::GradientMethod;

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
             const Eigen::Vector3d& upper) {
            self.setLinearBounds(lower, upper);
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setAngularBounds",
          [](ErrorMethod& self,
             const Eigen::Vector3d& lower,
             const Eigen::Vector3d& upper) {
            self.setAngularBounds(lower, upper);
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setBounds",
          [](ErrorMethod& self,
             const Eigen::Vector6d& lower,
             const Eigen::Vector6d& upper) { self.setBounds(lower, upper); },
          nb::arg("lower"),
          nb::arg("upper"));

  nb::class_<GradientMethod>(m, "InverseKinematicsGradientMethod")
      .def(
          "setComponentWeights",
          &GradientMethod::setComponentWeights,
          nb::arg("weights"))
      .def("getComponentWeights", [](const GradientMethod& self) {
        return self.getComponentWeights();
      });

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
          [](IK& self, const std::shared_ptr<dart::math::Solver>& solver) {
            self.setSolver(solver);
          },
          nb::arg("solver"),
          nb::keep_alive<1, 2>())
      .def("getProblem", [](IK& self) { return self.getProblem(); })
      .def(
          "getGradientMethod",
          [](IK& self) -> GradientMethod& { return self.getGradientMethod(); },
          nb::rv_policy::reference_internal)
      .def(
          "solveAndApply",
          [](IK& self, bool allowIncompleteResult) {
            return self.solveAndApply(allowIncompleteResult);
          },
          nb::arg("allow_incomplete_result") = true)
      .def("__repr__", [](const IK& self) {
        const auto target = self.getTarget();
        const auto* node = self.getNode();
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back(
            "node", node ? repr_string(node->getName()) : "None");
        fields.emplace_back(
            "target", target ? repr_string(target->getName()) : "None");
        fields.emplace_back("active", repr_bool(self.isActive()));
        fields.emplace_back("dofs", std::to_string(self.getDofs().size()));
        return format_repr("InverseKinematics", fields);
      });

  m.def(
      "createSimpleFrame",
      [](dart::dynamics::Frame* parent,
         const std::string& name,
         const Eigen::Isometry3d& tf) {
        dart::dynamics::Frame* resolved
            = parent ? parent : dart::dynamics::Frame::World();
        return std::make_shared<dart::dynamics::SimpleFrame>(
            resolved, name, tf);
      },
      nb::arg("parent"),
      nb::arg("name"),
      nb::arg("transform"));
}

DART_SUPPRESS_DEPRECATED_END

} // namespace dart::python_nb
