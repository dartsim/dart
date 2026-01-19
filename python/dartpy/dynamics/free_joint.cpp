#include "dynamics/free_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/free_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defFreeJoint(nb::module_& m)
{
  using FreeJoint = dart::dynamics::FreeJoint;
  using Properties = FreeJoint::Properties;

  nb::class_<Properties>(m, "FreeJointProperties")
      .def(nb::init<>())
      .def_rw("mName", &Properties::mName)
      .def_rw("mT_ParentBodyToJoint", &Properties::mT_ParentBodyToJoint)
      .def_rw("mT_ChildBodyToJoint", &Properties::mT_ChildBodyToJoint);

  nb::class_<FreeJoint, dart::dynamics::Joint>(m, "FreeJoint")
      .def(
          "setTransform",
          [](FreeJoint& self, const Eigen::Isometry3d& tf) {
            self.setTransform(tf);
          },
          nb::arg("transform"))
      .def(
          "getRelativeTransform",
          [](const FreeJoint& self) { return self.getRelativeTransform(); })
      .def(
          "setPositions",
          [](FreeJoint& self, const Eigen::Vector6d& positions) {
            self.setPositions(positions);
          },
          nb::arg("positions"));

  registerPolymorphicCaster<dart::dynamics::Joint, FreeJoint>();
}

} // namespace dart::python_nb
