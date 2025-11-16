#include "constraint/module.hpp"

#include "constraint/constraint_base.hpp"
#include "constraint/dynamic_joint_constraint.hpp"
#include "constraint/joint_constraint.hpp"
#include "constraint/joint_coulomb_friction_constraint.hpp"

namespace dart::python_nb {

void defConstraintModule(nanobind::module_& m)
{
  auto constraint = m.def_submodule("constraint");
  defConstraintBase(constraint);
  defDynamicJointConstraint(constraint);
  defJointConstraint(constraint);
  defJointCoulombFrictionConstraint(constraint);
}

} // namespace dart::python_nb
