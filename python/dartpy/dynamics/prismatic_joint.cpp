#include "dynamics/prismatic_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/prismatic_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defPrismaticJoint(nb::module_& m)
{
  using PrismaticJoint = dart::dynamics::PrismaticJoint;
  using Properties = PrismaticJoint::Properties;

  nb::class_<Properties>(m, "PrismaticJointProperties")
      .def(nb::init<>())
      .def_rw(
          "mAxis",
          &dart::dynamics::detail::PrismaticJointUniqueProperties::mAxis);

  nb::class_<PrismaticJoint, dart::dynamics::Joint>(m, "PrismaticJoint")
      .def("setAxis", &PrismaticJoint::setAxis, nb::arg("axis"))
      .def(
          "getAxis",
          [](const PrismaticJoint& self) -> const Eigen::Vector3d& {
            return self.getAxis();
          },
          nb::rv_policy::reference_internal);

  registerPolymorphicCaster<dart::dynamics::Joint, PrismaticJoint>();
}

} // namespace dart::python_nb
