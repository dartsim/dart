#include "dynamics/end_effector.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/end_effector.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defEndEffector(nb::module_& m)
{
  using Support = dart::dynamics::Support;
  using EndEffector = dart::dynamics::EndEffector;

  nb::class_<Support>(m, "Support")
      .def(
          "setGeometry",
          [](Support& self, const dart::math::SupportGeometry& geometry) {
            self.setGeometry(geometry);
          },
          nb::arg("geometry"))
      .def(
          "getGeometry", [](const Support& self) { return self.getGeometry(); })
      .def(
          "setActive",
          [](Support& self, bool supporting) { self.setActive(supporting); },
          nb::arg("supporting") = true)
      .def("isActive", [](const Support& self) { return self.isActive(); });

  nb::class_<EndEffector, dart::dynamics::JacobianNode>(m, "EndEffector")
      .def(
          "setDefaultRelativeTransform",
          [](EndEffector& self,
             const Eigen::Isometry3d& transform,
             bool useNow) {
            self.setDefaultRelativeTransform(transform, useNow);
          },
          nb::arg("transform"),
          nb::arg("use_now") = false)
      .def(
          "getName",
          [](const EndEffector& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "resetRelativeTransform",
          [](EndEffector& self) { self.resetRelativeTransform(); })
      .def(
          "setRelativeTransform",
          [](EndEffector& self, const Eigen::Isometry3d& transform) {
            self.setRelativeTransform(transform);
          },
          nb::arg("transform"))
      .def(
          "createSupport",
          [](EndEffector& self) { return self.createSupport(); },
          nb::rv_policy::reference_internal)
      .def(
          "getSupport",
          [](EndEffector& self) { return self.getSupport(); },
          nb::rv_policy::reference_internal)
      .def(
          "getSupport",
          [](EndEffector& self, bool createIfNull) {
            return self.getSupport(createIfNull);
          },
          nb::rv_policy::reference_internal,
          nb::arg("create_if_null"))
      .def(
          "hasSupport",
          [](const EndEffector& self) { return self.hasSupport(); })
      .def("removeSupport", [](EndEffector& self) { self.removeSupport(); })
      .def(
          "getWorldTransform",
          [](const EndEffector& self) { return self.getWorldTransform(); })
      .def(
          "getIK",
          [](EndEffector& self, bool createIfNull) {
            return self.getIK(createIfNull);
          },
          nb::arg("create_if_null") = false);

  registerPolymorphicCaster<dart::dynamics::Frame, EndEffector>();
  registerPolymorphicCaster<dart::dynamics::JacobianNode, EndEffector>();
}

} // namespace dart::python_nb
