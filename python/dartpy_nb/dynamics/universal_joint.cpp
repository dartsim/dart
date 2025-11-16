#include "dynamics/universal_joint.hpp"

#include "dart/dynamics/UniversalJoint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defUniversalJoint(nb::module_& m)
{
  using UniversalJoint = dart::dynamics::UniversalJoint;
  using Properties = UniversalJoint::Properties;

  nb::class_<Properties>(m, "UniversalJointProperties")
      .def(nb::init<>())
      .def(
          nb::init<const dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<2>>::Properties&>(),
          nb::arg("properties"));

  nb::class_<UniversalJoint, dart::dynamics::Joint>(m, "UniversalJoint")
      .def(
          "getUniversalJointProperties",
          [](const UniversalJoint& self) {
            return self.getUniversalJointProperties();
          })
      .def(
          "getType",
          [](const UniversalJoint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("isCyclic", &UniversalJoint::isCyclic, nb::arg("index"))
      .def(
          "setAxis1",
          [](UniversalJoint& self, const Eigen::Vector3d& axis) {
            self.setAxis1(axis);
          },
          nb::arg("axis"))
      .def(
          "setAxis2",
          [](UniversalJoint& self, const Eigen::Vector3d& axis) {
            self.setAxis2(axis);
          },
          nb::arg("axis"))
      .def(
          "getAxis1",
          [](const UniversalJoint& self) -> const Eigen::Vector3d& {
            return self.getAxis1();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getAxis2",
          [](const UniversalJoint& self) -> const Eigen::Vector3d& {
            return self.getAxis2();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getRelativeJacobianStatic",
          [](const UniversalJoint& self, const Eigen::Vector2d& positions) {
            return self.getRelativeJacobianStatic(positions);
          },
          nb::arg("positions"))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return UniversalJoint::getStaticType();
          },
          nb::rv_policy::reference);
}

} // namespace dart::python_nb
