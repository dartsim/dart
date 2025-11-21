#include "dynamics/planar_joint.hpp"

#include "dart/dynamics/PlanarJoint.hpp"

#include <nanobind/nanobind.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defPlanarJoint(nb::module_& m)
{
  using PlanarJoint = dart::dynamics::PlanarJoint;

  nb::class_<PlanarJoint, dart::dynamics::Joint>(m, "PlanarJoint");

  registerPolymorphicCaster<dart::dynamics::Joint, PlanarJoint>();
}

} // namespace dart::python_nb
