#include "constraint/module.hpp"

#include "constraint/constraint_base.hpp"
#include "constraint/dynamic_joint_constraint.hpp"
#include "constraint/joint_constraint.hpp"
#include "constraint/joint_coulomb_friction_constraint.hpp"

namespace dart::python_nb {

void defConstraintModule(nanobind::module_& m)
{
  defConstraintBase(m);
  defDynamicJointConstraint(m);
  defJointConstraint(m);
  defJointCoulombFrictionConstraint(m);
}

} // namespace dart::python_nb
