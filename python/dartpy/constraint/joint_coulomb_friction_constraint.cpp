#include "constraint/joint_coulomb_friction_constraint.hpp"

#include "dart/constraint/JointCoulombFrictionConstraint.hpp"
#include "dart/dynamics/Joint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJointCoulombFrictionConstraint(nb::module_& m)
{
  using Constraint = dart::constraint::JointCoulombFrictionConstraint;

  nb::class_<Constraint, dart::constraint::ConstraintBase>(
      m, "JointCoulombFrictionConstraint")
      .def(nb::init<dart::dynamics::Joint*>(), nb::arg("joint"))
      .def_static(
          "setConstraintForceMixing",
          &Constraint::setConstraintForceMixing,
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing", &Constraint::getConstraintForceMixing);
}

} // namespace dart::python_nb
