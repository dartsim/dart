#include "constraint/dynamic_joint_constraint.hpp"

#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/DynamicJointConstraint.hpp"
#include "dart/constraint/RevoluteJointConstraint.hpp"
#include "dart/constraint/WeldJointConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defDynamicJointConstraint(nb::module_& m)
{
  using DynamicJointConstraint = dart::constraint::DynamicJointConstraint;
  using BallJointConstraint = dart::constraint::BallJointConstraint;
  using RevoluteJointConstraint = dart::constraint::RevoluteJointConstraint;
  using WeldJointConstraint = dart::constraint::WeldJointConstraint;

  nb::class_<DynamicJointConstraint, dart::constraint::ConstraintBase>(
      m, "DynamicJointConstraint")
      .def_static(
          "setErrorAllowance",
          &DynamicJointConstraint::setErrorAllowance,
          nb::arg("allowance"))
      .def_static(
          "getErrorAllowance", &DynamicJointConstraint::getErrorAllowance)
      .def_static(
          "setErrorReductionParameter",
          &DynamicJointConstraint::setErrorReductionParameter,
          nb::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          &DynamicJointConstraint::getErrorReductionParameter)
      .def_static(
          "setMaxErrorReductionVelocity",
          &DynamicJointConstraint::setMaxErrorReductionVelocity,
          nb::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          &DynamicJointConstraint::getMaxErrorReductionVelocity)
      .def_static(
          "setConstraintForceMixing",
          &DynamicJointConstraint::setConstraintForceMixing,
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing",
          &DynamicJointConstraint::getConstraintForceMixing);

  nb::class_<BallJointConstraint, DynamicJointConstraint>(
      m, "BallJointConstraint")
      .def(
          nb::init<dart::dynamics::BodyNode*, const Eigen::Vector3d&>(),
          nb::arg("bodyNode"),
          nb::arg("jointPosition"))
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&>(),
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"),
          nb::arg("jointPosition"))
      .def(
          "getType",
          [](const BallJointConstraint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return BallJointConstraint::getStaticType();
          },
          nb::rv_policy::reference_internal);

  nb::class_<RevoluteJointConstraint, DynamicJointConstraint>(
      m, "RevoluteJointConstraint")
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("bodyNode"),
          nb::arg("jointPosition"),
          nb::arg("axis"))
      .def(
          nb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"),
          nb::arg("jointPosition"),
          nb::arg("axis1"),
          nb::arg("axis2"))
      .def(
          "getType",
          [](const RevoluteJointConstraint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return RevoluteJointConstraint::getStaticType();
          },
          nb::rv_policy::reference_internal);

  nb::class_<WeldJointConstraint, DynamicJointConstraint>(
      m, "WeldJointConstraint")
      .def(nb::init<dart::dynamics::BodyNode*>(), nb::arg("bodyNode"))
      .def(
          nb::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"))
      .def(
          "setRelativeTransform",
          [](WeldJointConstraint& self, const Eigen::Isometry3d& tf) {
            self.setRelativeTransform(tf);
          },
          nb::arg("transform"))
      .def(
          "getRelativeTransform",
          [](const WeldJointConstraint& self) -> const Eigen::Isometry3d& {
            return self.getRelativeTransform();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getType",
          [](const WeldJointConstraint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return WeldJointConstraint::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
