/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "include_pybind11.h"
#include "include_pybind11.h"

#include "dart/math/all.hpp"

namespace py = pybind11;

namespace dart::python {

#define DARTPY_VECTOR_SPACE(dim)                                               \
  auto r##dim = ::py::class_<math::R##dim##d>(m, "R" #dim)                     \
                    .def(::py::init<>())                                       \
                    .def(                                                      \
                        "is_zero",                                             \
                        &math::R##dim##d::is_zero,                             \
                        py::arg("tolerance") = math::eps<double>())            \
                    .def("is_identity", &math::R##dim##d::is_identity);        \
  (void)r##dim;                                                                \
  auto r##dim##_tangent                                                        \
      = ::py::class_<math::R##dim##Tangentd>(m, "R" #dim "Tangent")            \
            .def(::py::init<>())                                               \
            .def(                                                              \
                "is_zero",                                                     \
                &math::R##dim##Tangentd::is_zero,                              \
                py::arg("tolerance") = math::eps<double>());                   \
  (void)r##dim##_tangent;                                                      \
  auto r##dim##_cotangent                                                      \
      = ::py::class_<math::R##dim##Cotangentd>(m, "R" #dim "Cotangent")        \
            .def(::py::init<>())                                               \
            .def(                                                              \
                "is_zero",                                                     \
                &math::R##dim##Cotangentd::is_zero,                            \
                py::arg("tolerance") = math::eps<double>());                   \
  (void)r##dim##_cotangent

void py_r(py::module& m)
{
  // Vector space (dynamic size)
  ::py::class_<math::RXd>(m, "R")
      .def(::py::init<int>(), py::arg("size") = 0)
      .def(
          "is_zero",
          &math::RXd::is_zero,
          py::arg("tolerance") = math::eps<double>());
  ::py::class_<math::RTangent<double, ::Eigen::Dynamic>>(m, "RTangent")
      .def(::py::init<int>(), py::arg("size") = 0)
      .def(
          "is_zero",
          &math::RTangent<double, ::Eigen::Dynamic>::is_zero,
          py::arg("tolerance") = math::eps<double>());
  ::py::class_<math::RCotangent<double, ::Eigen::Dynamic>>(m, "RCotangent")
      .def(::py::init<int>(), py::arg("size") = 0)
      .def(
          "is_zero",
          &math::RCotangent<double, ::Eigen::Dynamic>::is_zero,
          py::arg("tolerance") = math::eps<double>());

  // Vector spaces (fixed size)
  DARTPY_VECTOR_SPACE(0);
  DARTPY_VECTOR_SPACE(1);
  DARTPY_VECTOR_SPACE(2);
  DARTPY_VECTOR_SPACE(3);
  DARTPY_VECTOR_SPACE(4);
  DARTPY_VECTOR_SPACE(5);
  DARTPY_VECTOR_SPACE(6);
}

} // namespace dart::python
