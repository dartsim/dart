#include "dynamics/translational_joint2d.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/translational_joint2_d.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defTranslationalJoint2D(nb::module_& m)
{
  using TranslationalJoint2D = dart::dynamics::TranslationalJoint2D;
  using Properties = TranslationalJoint2D::Properties;

  nb::class_<Properties>(m, "TranslationalJoint2DProperties")
      .def(nb::init<>())
      .def(
          nb::init<const dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<2>>::Properties&>(),
          nb::arg("properties"));

  nb::class_<TranslationalJoint2D, dart::dynamics::Joint>(
      m, "TranslationalJoint2D")
      .def(
          "getTranslationalJoint2DProperties",
          [](const TranslationalJoint2D& self) {
            return self.getTranslationalJoint2DProperties();
          })
      .def(
          "getType",
          [](const TranslationalJoint2D& self) {
            return std::string(self.getType());
          })
      .def("isCyclic", &TranslationalJoint2D::isCyclic, nb::arg("index"))
      .def(
          "setXYPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setXYPlane(rename_dofs);
          },
          nb::arg("rename_dofs") = true)
      .def(
          "setYZPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setYZPlane(rename_dofs);
          },
          nb::arg("rename_dofs") = true)
      .def(
          "setZXPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setZXPlane(rename_dofs);
          },
          nb::arg("rename_dofs") = true)
      .def(
          "setArbitraryPlane",
          [](TranslationalJoint2D& self,
             const Eigen::Vector3d& axis1,
             const Eigen::Vector3d& axis2,
             bool rename_dofs) {
            self.setArbitraryPlane(axis1, axis2, rename_dofs);
          },
          nb::arg("trans_axis1"),
          nb::arg("trans_axis2"),
          nb::arg("rename_dofs") = true)
      .def_static("getStaticType", []() {
        return std::string(TranslationalJoint2D::getStaticType());
      });

  registerPolymorphicCaster<dart::dynamics::Joint, TranslationalJoint2D>();
}

} // namespace dart::python_nb
