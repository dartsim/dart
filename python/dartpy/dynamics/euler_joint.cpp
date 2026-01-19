#include "dynamics/euler_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/euler_joint.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defEulerJoint(nb::module_& m)
{
  using EulerJoint = dart::dynamics::EulerJoint;

  nb::class_<EulerJoint, dart::dynamics::Joint>(m, "EulerJoint");

  registerPolymorphicCaster<dart::dynamics::Joint, EulerJoint>();
}

} // namespace dart::python_nb
