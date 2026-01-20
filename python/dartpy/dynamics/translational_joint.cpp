#include "dynamics/translational_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/translational_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defTranslationalJoint(nb::module_& m)
{
  using TranslationalJoint = dart::dynamics::TranslationalJoint;
  using Properties = TranslationalJoint::Properties;

  nb::class_<Properties>(m, "TranslationalJointProperties")
      .def(nb::init<>())
      .def(
          nb::init<const dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<3>>::Properties&>(),
          nb::arg("properties"));

  nb::class_<TranslationalJoint, dart::dynamics::Joint>(m, "TranslationalJoint")
      .def(
          "getTranslationalJointProperties",
          [](const TranslationalJoint& self) {
            return self.getTranslationalJointProperties();
          })
      .def(
          "getType",
          [](const TranslationalJoint& self) {
            return std::string(self.getType());
          })
      .def("isCyclic", &TranslationalJoint::isCyclic, nb::arg("index"))
      .def(
          "getRelativeJacobianStatic",
          [](const TranslationalJoint& self, const Eigen::Vector3d& positions) {
            return self.getRelativeJacobianStatic(positions);
          },
          nb::arg("positions"))
      .def_static("getStaticType", []() {
        return std::string(TranslationalJoint::getStaticType());
      });

  registerPolymorphicCaster<dart::dynamics::Joint, TranslationalJoint>();
}

} // namespace dart::python_nb
