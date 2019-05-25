/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

// TODO(JS): For some reason, passing const reference causes segfault errors.
// Use "pass-by-value" for now.
#define DARTPY_DEFINE_EULAERTOMATRIX(order)                                    \
  m.def(                                                                       \
      "euler" #order "ToMatrix",                                               \
      +[](Eigen::Vector3d angle) -> Eigen::Matrix3d {                          \
        return dart::math::euler##order##ToMatrix(angle);                      \
      },                                                                       \
      ::pybind11::arg("angle"));

#define DARTPY_DEFINE_MATRIXTOEULAER(order)                                    \
  m.def(                                                                       \
      "matrixToEuler" #order,                                                  \
      +[](const Eigen::Matrix3d& R) -> Eigen::Vector3d {                       \
        return dart::math::matrixToEuler##order(R);                            \
      },                                                                       \
      ::pybind11::arg("R"));

namespace dart {
namespace python {

void Geometry(pybind11::module& m)
{
  DARTPY_DEFINE_EULAERTOMATRIX(XYX);
  DARTPY_DEFINE_EULAERTOMATRIX(XYZ);
  DARTPY_DEFINE_EULAERTOMATRIX(XZX);
  DARTPY_DEFINE_EULAERTOMATRIX(XZY);
  DARTPY_DEFINE_EULAERTOMATRIX(YXY);
  DARTPY_DEFINE_EULAERTOMATRIX(YXZ);
  DARTPY_DEFINE_EULAERTOMATRIX(YZX);
  DARTPY_DEFINE_EULAERTOMATRIX(YZY);
  DARTPY_DEFINE_EULAERTOMATRIX(ZXY);
  DARTPY_DEFINE_EULAERTOMATRIX(ZYX);
  DARTPY_DEFINE_EULAERTOMATRIX(ZXZ);
  DARTPY_DEFINE_EULAERTOMATRIX(ZYZ);

  DARTPY_DEFINE_MATRIXTOEULAER(XYX);
  DARTPY_DEFINE_MATRIXTOEULAER(XYZ);
  // DARTPY_DEFINE_MATRIXTOEULAER(XZX);
  DARTPY_DEFINE_MATRIXTOEULAER(XZY);
  // DARTPY_DEFINE_MATRIXTOEULAER(YXY);
  DARTPY_DEFINE_MATRIXTOEULAER(YXZ);
  DARTPY_DEFINE_MATRIXTOEULAER(YZX);
  // DARTPY_DEFINE_MATRIXTOEULAER(YZY);
  DARTPY_DEFINE_MATRIXTOEULAER(ZXY);
  DARTPY_DEFINE_MATRIXTOEULAER(ZYX);
  // DARTPY_DEFINE_MATRIXTOEULAER(ZXZ);
  // DARTPY_DEFINE_MATRIXTOEULAER(ZYZ);

  m.def(
      "expMap",
      +[](const Eigen::Vector6d& _S) -> Eigen::Isometry3d {
        return dart::math::expMap(_S);
      },
      ::pybind11::arg("S"));

  m.def(
      "expMapJac",
      +[](const Eigen::Vector3d& _expmap) -> Eigen::Matrix3d {
        return dart::math::expMapJac(_expmap);
      },
      ::pybind11::arg("expmap"));

  m.def(
      "expMapRot",
      +[](const Eigen::Vector3d& _expmap) -> Eigen::Matrix3d {
        return dart::math::expMapRot(_expmap);
      },
      ::pybind11::arg("expmap"));

  m.def(
      "expToQuat",
      +[](const Eigen::Vector3d& _v) -> Eigen::Quaterniond {
        return dart::math::expToQuat(_v);
      },
      ::pybind11::arg("v"));

  m.def(
      "quatToExp",
      +[](const Eigen::Quaterniond& _q) -> Eigen::Vector3d {
        return dart::math::quatToExp(_q);
      },
      ::pybind11::arg("q"));

  m.def(
      "expAngular",
      +[](const Eigen::Vector3d& _s) -> Eigen::Isometry3d {
        return dart::math::expAngular(_s);
      },
      ::pybind11::arg("s"));

  m.def(
      "verifyRotation",
      +[](const Eigen::Matrix3d& _R) -> bool {
        return dart::math::verifyRotation(_R);
      },
      ::pybind11::arg("R"));

  m.def(
      "verifyTransform",
      +[](const Eigen::Isometry3d& _T) -> bool {
        return dart::math::verifyTransform(_T);
      },
      ::pybind11::arg("T"));
}

} // namespace python
} // namespace dart
