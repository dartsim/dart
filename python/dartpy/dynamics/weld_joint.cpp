#include "dynamics/weld_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/weld_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defWeldJoint(nb::module_& m)
{
  using WeldJoint = dart::dynamics::WeldJoint;

  nb::class_<WeldJoint, dart::dynamics::ZeroDofJoint>(m, "WeldJoint")
      .def(
          "getWeldJointProperties",
          [](const WeldJoint& self) { return self.getWeldJointProperties(); })
      .def(
          "getType",
          [](const WeldJoint& self) { return std::string(self.getType()); })
      .def(
          "isCyclic",
          [](const WeldJoint& self, std::size_t index) {
            return self.isCyclic(index);
          },
          nb::arg("index"))
      .def(
          "setTransformFromParentBodyNode",
          [](WeldJoint& self, const Eigen::Isometry3d& tf) {
            self.setTransformFromParentBodyNode(tf);
          },
          nb::arg("transform"))
      .def(
          "setTransformFromChildBodyNode",
          [](WeldJoint& self, const Eigen::Isometry3d& tf) {
            self.setTransformFromChildBodyNode(tf);
          },
          nb::arg("transform"))
      .def_static("getStaticType", []() {
        return std::string(WeldJoint::getStaticType());
      });

  registerPolymorphicCaster<dart::dynamics::Joint, WeldJoint>();
}

} // namespace dart::python_nb
