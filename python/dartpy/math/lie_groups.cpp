/*
 * Copyright (c) 2011, The DART development contributors
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

#include "math/lie_groups.hpp"

#include "dart/math/lie_groups.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

#include <sstream>
#include <string>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

template <typename Derived>
std::string matrixToString(const Eigen::MatrixBase<Derived>& m)
{
  std::ostringstream ss;
  ss << m;
  return ss.str();
}

template <typename Derived>
std::string matrixToRepr(const char* name, const Eigen::MatrixBase<Derived>& m)
{
  std::ostringstream ss;
  ss << name << "(\n" << m << "\n)";
  return ss.str();
}

} // namespace

void defLieGroups(nb::module_& m)
{
  using namespace ::dart::math;

  const double tol = LieGroupTol<double>();

  auto so3 = nb::class_<SO3d>(m, "SO3");
  so3.def(nb::init<>())
      .def(
          "__eq__",
          [](const SO3d& self, const SO3d& other) { return self == other; })
      .def(
          "__ne__",
          [](const SO3d& self, const SO3d& other) { return self != other; })
      .def(
          "__mul__",
          [](const SO3d& self, const SO3d& other) -> SO3d {
            return self * other;
          })
      .def(
          "__str__", [](const SO3d& x) { return matrixToString(x.toMatrix()); })
      .def(
          "__repr__",
          [](const SO3d& x) { return matrixToRepr("SO3", x.toMatrix()); })
      .def("set_random", [](SO3d& self) { self.setRandom(); })
      .def(
          "is_approx",
          [](const SO3d& self, const SO3d& other, double t) {
            return self.isApprox(other, t);
          },
          nb::arg("other"),
          nb::arg("tol") = tol)
      .def(
          "is_identity",
          [](const SO3d& self, double t) { return self.isIdentity(t); },
          nb::arg("tol") = tol)
      .def("inverse", [](const SO3d& self) -> SO3d { return self.inverse(); })
      .def(
          "log",
          [](const SO3d& self, double t) -> SO3Tangentd { return self.log(t); },
          nb::arg("tol") = tol)
      .def(
          "inverse_in_place",
          [](SO3d& self) -> SO3d& {
            self.inverseInPlace();
            return self;
          },
          nb::rv_policy::reference_internal)
      .def("to_matrix", [](const SO3d& self) { return self.toMatrix(); });

  auto se3 = nb::class_<SE3d>(m, "SE3");
  se3.def(nb::init<>())
      .def(nb::init<const SO3d&, const Eigen::Vector3d&>())
      .def(
          "__eq__",
          [](const SE3d& self, const SE3d& other) { return self == other; })
      .def(
          "__ne__",
          [](const SE3d& self, const SE3d& other) { return self != other; })
      .def(
          "__mul__",
          [](const SE3d& self, const SE3d& other) -> SE3d {
            return self * other;
          })
      .def(
          "__str__", [](const SE3d& x) { return matrixToString(x.toMatrix()); })
      .def(
          "__repr__",
          [](const SE3d& x) { return matrixToRepr("SE3", x.toMatrix()); })
      .def(
          "is_approx",
          [](const SE3d& self, const SE3d& other, double t) {
            return self.isApprox(other, t);
          },
          nb::arg("other"),
          nb::arg("tol") = tol)
      .def(
          "is_identity",
          [](const SE3d& self, double t) { return self.isIdentity(t); },
          nb::arg("tol") = tol)
      .def("inverse", [](const SE3d& self) -> SE3d { return self.inverse(); })
      .def(
          "log",
          [](const SE3d& self, double t) -> SE3Tangentd { return self.log(t); },
          nb::arg("tol") = tol)
      .def(
          "inverse_in_place",
          [](SE3d& self) -> SE3d& {
            self.inverseInPlace();
            return self;
          },
          nb::rv_policy::reference_internal)
      .def("to_matrix", [](const SE3d& self) { return self.toMatrix(); });

  m.def("rand_so3", []() { return SO3d::Random(); });
  m.def("rand_se3", []() { return SE3d::Random(); });

  auto so3_tangent = nb::class_<SO3Tangentd>(m, "SO3Tangent");
  so3_tangent.def(nb::init<>())
      .def_static("zero", []() { return SO3Tangentd::Zero(); })
      .def_static("random", []() { return SO3Tangentd::Random(); })
      .def(
          "__eq__",
          [](const SO3Tangentd& self, const SO3Tangentd& other) {
            return self == other;
          })
      .def(
          "__ne__",
          [](const SO3Tangentd& self, const SO3Tangentd& other) {
            return self != other;
          })
      .def(
          "__str__",
          [](const SO3Tangentd& x) {
            return matrixToString(x.params().transpose());
          })
      .def(
          "__repr__",
          [](const SO3Tangentd& x) {
            return matrixToRepr("SO3Tangent", x.params().transpose());
          })
      .def(
          "is_approx",
          [](const SO3Tangentd& self, const SO3Tangentd& other, double t) {
            return self.isApprox(other, t);
          },
          nb::arg("other"),
          nb::arg("tol") = tol)
      .def(
          "is_zero",
          [](const SO3Tangentd& self, double t) { return self.isZero(t); },
          nb::arg("tol") = tol)
      .def(
          "exp",
          [](const SO3Tangentd& self, double t) -> SO3d { return self.exp(t); },
          nb::arg("tol") = tol);

  auto se3_tangent = nb::class_<SE3Tangentd>(m, "SE3Tangent");
  se3_tangent.def(nb::init<>())
      .def_static("zero", []() { return SE3Tangentd::Zero(); })
      .def_static("random", []() { return SE3Tangentd::Random(); })
      .def(
          "__eq__",
          [](const SE3Tangentd& self, const SE3Tangentd& other) {
            return self == other;
          })
      .def(
          "__ne__",
          [](const SE3Tangentd& self, const SE3Tangentd& other) {
            return self != other;
          })
      .def(
          "__str__",
          [](const SE3Tangentd& x) {
            return matrixToString(x.params().transpose());
          })
      .def(
          "__repr__",
          [](const SE3Tangentd& x) {
            return matrixToRepr("SE3Tangent", x.params().transpose());
          })
      .def(
          "is_approx",
          [](const SE3Tangentd& self, const SE3Tangentd& other, double t) {
            return self.isApprox(other, t);
          },
          nb::arg("other"),
          nb::arg("tol") = tol)
      .def(
          "is_zero",
          [](const SE3Tangentd& self, double t) { return self.isZero(t); },
          nb::arg("tol") = tol)
      .def(
          "exp",
          [](const SE3Tangentd& self, double t) -> SE3d { return self.exp(t); },
          nb::arg("tol") = tol);

  m.def("rand_so3_tangent", []() { return SO3Tangentd::Random(); });
  m.def("rand_se3_tangent", []() { return SE3Tangentd::Random(); });

  so3.attr("Tangent") = so3_tangent;
  se3.attr("Tangent") = se3_tangent;

  m.def(
      "exp",
      [](const SO3Tangentd& dx, double t) -> SO3d { return Exp(dx, t); },
      nb::arg("dx"),
      nb::arg("tol") = tol);
  m.def(
      "log",
      [](const SO3d& x, double t) -> SO3Tangentd { return Log(x, t); },
      nb::arg("x"),
      nb::arg("tol") = tol);
  m.def(
      "exp",
      [](const SE3Tangentd& dx, double t) -> SE3d { return Exp(dx, t); },
      nb::arg("dx"),
      nb::arg("tol") = tol);
  m.def(
      "log",
      [](const SE3d& x, double t) -> SE3Tangentd { return Log(x, t); },
      nb::arg("x"),
      nb::arg("tol") = tol);
}

} // namespace dart::python_nb
