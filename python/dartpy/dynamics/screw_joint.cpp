#include "dynamics/screw_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/ScrewJoint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defScrewJoint(nb::module_& m)
{
  using ScrewJoint = dart::dynamics::ScrewJoint;
  using Properties = ScrewJoint::Properties;

  nb::class_<Properties>(m, "ScrewJointProperties")
      .def(nb::init<>())
      .def_rw(
          "mAxis", &dart::dynamics::detail::ScrewJointUniqueProperties::mAxis)
      .def_rw(
          "mPitch",
          &dart::dynamics::detail::ScrewJointUniqueProperties::mPitch);

  nb::class_<ScrewJoint, dart::dynamics::Joint>(m, "ScrewJoint")
      .def(
          "getType",
          [](const ScrewJoint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("isCyclic", &ScrewJoint::isCyclic, nb::arg("index"))
      .def(
          "setAxis",
          [](ScrewJoint& self, const Eigen::Vector3d& axis) {
            self.setAxis(axis);
          },
          nb::arg("axis"))
      .def(
          "getAxis",
          [](const ScrewJoint& self) -> const Eigen::Vector3d& {
            return self.getAxis();
          },
          nb::rv_policy::reference_internal)
      .def("setPitch", &ScrewJoint::setPitch, nb::arg("pitch"))
      .def("getPitch", &ScrewJoint::getPitch)
      .def_static(
          "getStaticType",
          []() -> const std::string& { return ScrewJoint::getStaticType(); },
          nb::rv_policy::reference);

  registerPolymorphicCaster<dart::dynamics::Joint, ScrewJoint>();
}

} // namespace dart::python_nb
