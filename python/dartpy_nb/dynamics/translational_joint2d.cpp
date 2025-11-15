#include "dynamics/translational_joint2d.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/TranslationalJoint2D.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defTranslationalJoint2D(nb::module_& m)
{
  using TranslationalJoint2D = dart::dynamics::TranslationalJoint2D;
  using Properties = TranslationalJoint2D::Properties;

  nb::class_<Properties>(m, "TranslationalJoint2DProperties")
      .def(nb::init<>())
      .def(nb::init<const dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>::Properties&>(),
          nb::arg("properties"));

  nb::class_<TranslationalJoint2D, dart::dynamics::Joint, std::shared_ptr<TranslationalJoint2D>>(m, "TranslationalJoint2D")
      .def("getTranslationalJoint2DProperties",
          [](const TranslationalJoint2D& self) {
            return self.getTranslationalJoint2DProperties();
          })
      .def("getType",
          [](const TranslationalJoint2D& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("isCyclic", &TranslationalJoint2D::isCyclic, nb::arg("index"))
      .def("setXYPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setXYPlane(rename_dofs);
          },
          nb::arg("renameDofs") = true)
      .def("setYZPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setYZPlane(rename_dofs);
          },
          nb::arg("renameDofs") = true)
      .def("setZXPlane",
          [](TranslationalJoint2D& self, bool rename_dofs) {
            self.setZXPlane(rename_dofs);
          },
          nb::arg("renameDofs") = true)
      .def("setArbitraryPlane",
          [](TranslationalJoint2D& self,
              const Eigen::Vector3d& axis1,
              const Eigen::Vector3d& axis2,
              bool rename_dofs) {
            self.setArbitraryPlane(axis1, axis2, rename_dofs);
          },
          nb::arg("transAxis1"),
          nb::arg("transAxis2"),
          nb::arg("renameDofs") = true)
      .def_static("getStaticType",
          []() -> const std::string& {
            return TranslationalJoint2D::getStaticType();
          },
          nb::rv_policy::reference);
}

} // namespace dart::python_nb
