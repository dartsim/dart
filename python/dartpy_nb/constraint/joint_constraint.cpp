#include "constraint/joint_constraint.hpp"

#include "dart/constraint/JointConstraint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJointConstraint(nb::module_& m)
{
  using JointConstraint = dart::constraint::JointConstraint;

  nb::class_<JointConstraint, dart::constraint::ConstraintBase>(
      m, "JointConstraint")
      .def(nb::init<dart::dynamics::Joint*>(), nb::arg("joint"))
      .def_static(
          "setErrorAllowance",
          &JointConstraint::setErrorAllowance,
          nb::arg("allowance"))
      .def_static("getErrorAllowance", &JointConstraint::getErrorAllowance)
      .def_static(
          "setErrorReductionParameter",
          &JointConstraint::setErrorReductionParameter,
          nb::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          &JointConstraint::getErrorReductionParameter)
      .def_static(
          "setMaxErrorReductionVelocity",
          &JointConstraint::setMaxErrorReductionVelocity,
          nb::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          &JointConstraint::getMaxErrorReductionVelocity)
      .def_static(
          "setConstraintForceMixing",
          &JointConstraint::setConstraintForceMixing,
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing",
          &JointConstraint::getConstraintForceMixing);
}

} // namespace dart::python_nb
