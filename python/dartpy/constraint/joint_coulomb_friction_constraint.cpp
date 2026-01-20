#include "constraint/joint_coulomb_friction_constraint.hpp"

#include "common/type_casters.hpp"
#include "dart/constraint/joint_coulomb_friction_constraint.hpp"
#include "dart/dynamics/joint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJointCoulombFrictionConstraint(nb::module_& m)
{
  using Constraint = dart::constraint::JointCoulombFrictionConstraint;

  nb::class_<Constraint, dart::constraint::ConstraintBase>(
      m, "JointCoulombFrictionConstraint")
      .def(
          nb::new_([](dart::dynamics::Joint& joint) {
            return new Constraint(&joint);
          }),
          nb::arg("joint"),
          nb::keep_alive<1, 2>())
      .def_static(
          "setConstraintForceMixing",
          &Constraint::setConstraintForceMixing,
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing", &Constraint::getConstraintForceMixing);
}

} // namespace dart::python_nb
