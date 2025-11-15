#include "dynamics/inertia.hpp"

#include "dart/dynamics/Inertia.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defInertia(nb::module_& m)
{
  using Inertia = dart::dynamics::Inertia;

  nb::class_<Inertia>(m, "Inertia")
      .def(
          nb::init<double, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
          nb::arg("mass") = 1.0,
          nb::arg("com") = Eigen::Vector3d::Zero(),
          nb::arg("momentOfInertia") = Eigen::Matrix3d::Identity())
      .def(nb::init<const Eigen::Matrix6d&>(), nb::arg("spatialInertiaTensor"))
      .def("setMass", &Inertia::setMass, nb::arg("mass"))
      .def("getMass", &Inertia::getMass)
      .def("setLocalCOM", &Inertia::setLocalCOM, nb::arg("com"))
      .def(
          "getLocalCOM",
          &Inertia::getLocalCOM,
          nb::rv_policy::reference_internal)
      .def(
          "setMoment",
          nb::overload_cast<const Eigen::Matrix3d&>(&Inertia::setMoment),
          nb::arg("moment"))
      .def("getMoment", &Inertia::getMoment)
      .def(
          "setSpatialTensor",
          nb::overload_cast<const Eigen::Matrix6d&>(&Inertia::setSpatialTensor),
          nb::arg("spatial"))
      .def(
          "setSpatialTensor",
          nb::overload_cast<const Eigen::Matrix6d&, bool>(
              &Inertia::setSpatialTensor),
          nb::arg("spatial"),
          nb::arg("printWarnings"))
      .def(
          "getSpatialTensor",
          &Inertia::getSpatialTensor,
          nb::rv_policy::reference_internal)
      .def("transformed", &Inertia::transformed, nb::arg("transform"))
      .def(
          "transform",
          &Inertia::transform,
          nb::arg("transform"),
          nb::rv_policy::reference_internal)
      .def(
          "verify",
          &Inertia::verify,
          nb::arg("printWarnings") = true,
          nb::arg("tolerance") = 1e-8)
      .def(
          "__eq__",
          [](const Inertia& self, const Inertia& other) {
            return self == other;
          })
      .def_static(
          "verifyMoment",
          &Inertia::verifyMoment,
          nb::arg("moment"),
          nb::arg("printWarnings") = true,
          nb::arg("tolerance") = 1e-8)
      .def_static(
          "verifySpatialTensor",
          &Inertia::verifySpatialTensor,
          nb::arg("spatial"),
          nb::arg("printWarnings") = true,
          nb::arg("tolerance") = 1e-8);
}

} // namespace dart::python_nb
