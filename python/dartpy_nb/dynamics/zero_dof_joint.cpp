#include "dynamics/zero_dof_joint.hpp"

#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/ZeroDofJoint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defZeroDofJoint(nb::module_& m)
{
  using ZeroDofJoint = dart::dynamics::ZeroDofJoint;

  nb::class_<ZeroDofJoint::Properties>(m, "ZeroDofJointProperties")
      .def(nb::init<>())
      .def(
          nb::init<const dart::dynamics::Joint::Properties&>(),
          nb::arg("properties"));

  nb::class_<ZeroDofJoint, dart::dynamics::Joint>(m, "ZeroDofJoint")
      .def(
          "getZeroDofJointProperties",
          [](const ZeroDofJoint& self) {
            return self.getZeroDofJointProperties();
          });
}

} // namespace dart::python_nb

