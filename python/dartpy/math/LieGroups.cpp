/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dartpy/math/eigen_geometry_pybind.h"
#include "dartpy/math/eigen_pybind.h"

#include <dart/math/math.hpp>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart::python {

using namespace math;

void LieGroups(py::module& m)
{
  auto so3
      = py::class_<SO3d>(m, "SO3")
            .def(py::init<>())
            .def_static("Identity", &SO3d::Identity)
            .def_static("Random", &SO3d::Random)
            .def(py::self == py::self)
            .def(py::self != py::self)
            .def(py::self * py::self)
            .def("setRandom", &SO3d::setRandom)
            .def(
                "isApprox",
                [](const SO3d& self, const SO3d& other, double tol) -> bool {
                  return self.isApprox(other, tol);
                },
                py::arg("other"),
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "isIdentity",
                &SO3d::isIdentity,
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "inverse",
                [](const SO3d& self) -> SO3d { return self.inverse(); })
            .def("inverseInPlace", &SO3d::inverseInPlace)
            .def("toMatrix", &SO3d::toMatrix);

  auto se3
      = py::class_<SE3d>(m, "SE3")
            .def(py::init<>())
            .def(py::init<const SO3d&, const Eigen::Vector3d&>())
            .def_static("Identity", &SE3d::Identity)
            .def_static("Random", &SE3d::Random)
            .def(py::self == py::self)
            .def(py::self != py::self)
            .def(py::self * py::self)
            .def(
                "isApprox",
                [](const SE3d& self, const SE3d& other, double tol) -> bool {
                  return self.isApprox(other, tol);
                },
                py::arg("other"),
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "isIdentity",
                &SE3d::isIdentity,
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "inverse",
                [](const SE3d& self) -> SE3d { return self.inverse(); })
            .def("inverseInPlace", &SE3d::inverseInPlace)
            .def("toMatrix", &SE3d::toMatrix);

  py::implicitly_convertible<Eigen::Matrix3d, SO3d>();
  py::implicitly_convertible<Eigen::Matrix4d, SE3d>();

  auto so3_tangent
      = py::class_<SO3Tangentd>(m, "SO3Tangent")
            .def(py::init<>())
            .def_static("Zero", &SO3Tangentd::Zero)
            .def_static("Random", &SO3Tangentd::Random)
            .def(py::self == py::self)
            .def(py::self != py::self)
            .def(
                "isApprox",
                [](const SO3Tangentd& self,
                   const SO3Tangentd& other,
                   double tol) -> bool { return self.isApprox(other, tol); },
                py::arg("other"),
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "isZero",
                &SO3Tangentd::isZero,
                py::arg("tol") = LieGroupTol<double>());

  auto se3_tangent
      = py::class_<SE3Tangentd>(m, "SE3Tangent")
            .def(py::init<>())
            .def_static("Zero", &SE3Tangentd::Zero)
            .def_static("Random", &SE3Tangentd::Random)
            .def(py::self == py::self)
            .def(py::self != py::self)
            .def(
                "isApprox",
                [](const SE3Tangentd& self,
                   const SE3Tangentd& other,
                   double tol) -> bool { return self.isApprox(other, tol); },
                py::arg("other"),
                py::arg("tol") = LieGroupTol<double>())
            .def(
                "isZero",
                &SE3Tangentd::isZero,
                py::arg("tol") = LieGroupTol<double>());

  so3.attr("Tangent") = so3_tangent;
  se3.attr("Tangent") = se3_tangent;
}

} // namespace dart::python
