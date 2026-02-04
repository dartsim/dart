#include "dynamics/zero_dof_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/zero_dof_joint.hpp"

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
      .def("getZeroDofJointProperties", [](const ZeroDofJoint& self) {
        return self.getZeroDofJointProperties();
      });

  registerPolymorphicCaster<dart::dynamics::Joint, ZeroDofJoint>();
}

} // namespace dart::python_nb
