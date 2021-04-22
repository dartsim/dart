/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void Inertia(py::module& m)
{
  ::py::class_<
      dart::dynamics::Inertia,
      std::shared_ptr<dart::dynamics::Inertia>>(m, "Inertia")
      .def(
          ::py::init<double, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
          ::py::arg_v("mass", 1),
          ::py::arg_v(
              "com", Eigen::Vector3d::Zero(), "Eigen::Vector3d::Zero()"),
          ::py::arg_v(
              "momentOfInertia",
              Eigen::Matrix3d::Identity(),
              "Eigen::Matrix3d::Identity()"))
      .def(
          ::py::init<const Eigen::Matrix6d&>(),
          ::py::arg("spatialInertiaTensor"))
      .def(
          "setMass",
          +[](dart::dynamics::Inertia* self, double mass) {
            self->setMass(mass);
          },
          ::py::arg("mass"))
      .def(
          "getMass",
          +[](const dart::dynamics::Inertia* self) -> double {
            return self->getMass();
          })
      .def(
          "setLocalCOM",
          +[](dart::dynamics::Inertia* self, const Eigen::Vector3d& com) {
            self->setLocalCOM(com);
          },
          ::py::arg("com"))
      .def(
          "getLocalCOM",
          +[](const dart::dynamics::Inertia* self) -> const Eigen::Vector3d& {
            return self->getLocalCOM();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setMoment",
          +[](dart::dynamics::Inertia* self, const Eigen::Matrix3d& moment) {
            self->setMoment(moment);
          },
          ::py::arg("moment"))
      .def(
          "getMoment",
          +[](const dart::dynamics::Inertia* self) -> Eigen::Matrix3d {
            return self->getMoment();
          })
      .def(
          "setSpatialTensor",
          +[](dart::dynamics::Inertia* self, const Eigen::Matrix6d& spatial) {
            self->setSpatialTensor(spatial);
          },
          ::py::arg("spatial"))
      .def(
          "getSpatialTensor",
          +[](const dart::dynamics::Inertia* self) -> const Eigen::Matrix6d& {
            return self->getSpatialTensor();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "verify",
          +[](const dart::dynamics::Inertia* self,
              bool printWarnings,
              double tolerance) -> bool {
            return self->verify(printWarnings, tolerance);
          },
          ::py::arg_v("printWarnings", true),
          ::py::arg_v("tolerance", 1e-8))
      .def(
          "__eq__",
          +[](const dart::dynamics::Inertia* self,
              const dart::dynamics::Inertia& other) -> bool {
            return self && *self == other;
          })
      .def_static(
          "verifyMoment",
          +[](const Eigen::Matrix3d& moment,
              bool printWarnings,
              double tolerance) -> bool {
            return dart::dynamics::Inertia::verifyMoment(
                moment, printWarnings, tolerance);
          },
          ::py::arg("moment"),
          ::py::arg_v("printWarnings", true),
          ::py::arg_v("tolerance", 1e-8))
      .def_static(
          "verifySpatialTensor",
          +[](const Eigen::Matrix6d& spatial,
              bool printWarnings = true,
              double tolerance = 1e-8) -> bool {
            return dart::dynamics::Inertia::verifySpatialTensor(
                spatial, printWarnings, tolerance);
          },
          ::py::arg("spatial"),
          ::py::arg_v("printWarnings", true),
          ::py::arg_v("tolerance", 1e-8));
}

} // namespace python
} // namespace dart
