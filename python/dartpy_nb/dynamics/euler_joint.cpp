#include "dynamics/euler_joint.hpp"

#include "dart/dynamics/EulerJoint.hpp"

#include <nanobind/nanobind.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defEulerJoint(nb::module_& m)
{
  using EulerJoint = dart::dynamics::EulerJoint;

  nb::class_<EulerJoint, dart::dynamics::Joint>(m, "EulerJoint");

  registerPolymorphicCaster<dart::dynamics::Joint, EulerJoint>();
}

} // namespace dart::python_nb
