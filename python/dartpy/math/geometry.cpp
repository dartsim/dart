/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
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

#include "math/geometry.hpp"

#include "common/eigen_utils.hpp"
#include "dart/math/geometry.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

#include <cstddef>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

Eigen::Vector3d toVec3(const nb::handle& h)
{
  return toVector3(h);
}

#define DARTPY_DEF_EULER_TO_MATRIX(order)                                      \
  m.def(                                                                       \
      "euler" #order "ToMatrix",                                               \
      [](const nb::handle& angle) {                                            \
        Eigen::Vector3d vec = toVec3(angle);                                   \
        return dart::math::euler##order##ToMatrix(vec);                        \
      },                                                                       \
      nb::arg("angle"));

#define DARTPY_DEF_MATRIX_TO_EULER(order)                                      \
  m.def(                                                                       \
      "matrixToEuler" #order,                                                  \
      [](const Eigen::Matrix3d& R) {                                           \
        return dart::math::matrixToEuler##order(R);                            \
      },                                                                       \
      nb::arg("rotation"));

} // namespace

void defGeometry(nb::module_& m)
{
  DARTPY_DEF_EULER_TO_MATRIX(XYX);
  DARTPY_DEF_EULER_TO_MATRIX(XYZ);
  DARTPY_DEF_EULER_TO_MATRIX(XZX);
  DARTPY_DEF_EULER_TO_MATRIX(XZY);
  DARTPY_DEF_EULER_TO_MATRIX(YXY);
  DARTPY_DEF_EULER_TO_MATRIX(YXZ);
  DARTPY_DEF_EULER_TO_MATRIX(YZX);
  DARTPY_DEF_EULER_TO_MATRIX(YZY);
  DARTPY_DEF_EULER_TO_MATRIX(ZXY);
  DARTPY_DEF_EULER_TO_MATRIX(ZYX);
  DARTPY_DEF_EULER_TO_MATRIX(ZXZ);
  DARTPY_DEF_EULER_TO_MATRIX(ZYZ);

  DARTPY_DEF_MATRIX_TO_EULER(XYX);
  DARTPY_DEF_MATRIX_TO_EULER(XYZ);
  // DARTPY_DEF_MATRIX_TO_EULER(XZX);
  DARTPY_DEF_MATRIX_TO_EULER(XZY);
  // DARTPY_DEF_MATRIX_TO_EULER(YXY);
  DARTPY_DEF_MATRIX_TO_EULER(YXZ);
  DARTPY_DEF_MATRIX_TO_EULER(YZX);
  // DARTPY_DEF_MATRIX_TO_EULER(YZY);
  DARTPY_DEF_MATRIX_TO_EULER(ZXY);
  DARTPY_DEF_MATRIX_TO_EULER(ZYX);
  // DARTPY_DEF_MATRIX_TO_EULER(ZXZ);
  // DARTPY_DEF_MATRIX_TO_EULER(ZYZ);

  m.def(
      "expMap",
      [](const Eigen::Vector6d& S) { return dart::math::expMap(S); },
      nb::arg("screw_axis"));

  m.def(
      "expMapJac",
      [](const Eigen::Vector3d& expmap) {
        return dart::math::expMapJac(expmap);
      },
      nb::arg("expmap"));

  m.def(
      "expMapRot",
      [](const Eigen::Vector3d& expmap) {
        return dart::math::expMapRot(expmap);
      },
      nb::arg("expmap"));

  m.def(
      "expToQuat",
      [](const Eigen::Vector3d& v) { return dart::math::expToQuat(v); },
      nb::arg("v"));

  m.def(
      "quatToExp",
      [](const Eigen::Quaterniond& q) { return dart::math::quatToExp(q); },
      nb::arg("q"));

  m.def(
      "expAngular",
      [](const Eigen::Vector3d& s) { return dart::math::expAngular(s); },
      nb::arg("s"));

  m.def(
      "verifyRotation",
      [](const Eigen::Matrix3d& R) { return dart::math::verifyRotation(R); },
      nb::arg("rotation"));

  m.def(
      "verifyTransform",
      [](const Eigen::Isometry3d& T) { return dart::math::verifyTransform(T); },
      nb::arg("transform"));
}

} // namespace dart::python_nb
