#include "simulation_experimental/joint_type.hpp"

#include "dart/simulation/experimental/comps/joint.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJointType(nb::module_& m)
{
  using JointType = dart::simulation::experimental::comps::JointType;

  nb::enum_<JointType>(m, "JointType")
      .value("Fixed", JointType::Fixed)
      .value("Revolute", JointType::Revolute)
      .value("Prismatic", JointType::Prismatic)
      .value("Screw", JointType::Screw)
      .value("Universal", JointType::Universal)
      .value("Ball", JointType::Ball)
      .value("Planar", JointType::Planar)
      .value("Free", JointType::Free);
}

} // namespace dart::python_nb
