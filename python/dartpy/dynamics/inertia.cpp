#include "dynamics/inertia.hpp"

#include "common/eigen_utils.hpp"
#include "dart/dynamics/inertia.hpp"

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
          nb::new_(
              [](double mass, const nb::handle& com, const nb::handle& moment) {
                const Eigen::Vector3d comVec
                    = com.is_none() ? Eigen::Vector3d::Zero() : toVector3(com);
                const Eigen::Matrix3d momentMat
                    = moment.is_none() ? Eigen::Matrix3d::Identity()
                                       : toMatrix3(moment);
                return Inertia(mass, comVec, momentMat);
              }),
          nb::arg("mass") = 1.0,
          nb::arg("com") = nb::none(),
          nb::arg("moment_of_inertia") = nb::none())
      .def(
          nb::init<const Eigen::Matrix6d&>(), nb::arg("spatial_inertia_tensor"))
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
          nb::arg("print_warnings"))
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
          nb::arg("print_warnings") = true,
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
          nb::arg("print_warnings") = true,
          nb::arg("tolerance") = 1e-8)
      .def_static(
          "verifySpatialTensor",
          &Inertia::verifySpatialTensor,
          nb::arg("spatial"),
          nb::arg("print_warnings") = true,
          nb::arg("tolerance") = 1e-8);
}

} // namespace dart::python_nb
