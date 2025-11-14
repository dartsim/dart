#include "constraint/module.hpp"

#include "constraint/dynamic_joint_constraint.hpp"

namespace dart::python_nb {

void defConstraintModule(nanobind::module_& m)
{
  auto constraint = m.def_submodule("constraint");
  defDynamicJointConstraint(constraint);
}

} // namespace dart::python_nb
