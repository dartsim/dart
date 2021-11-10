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

#include <pybind11/pybind11.h>

#include "dart/math/all.hpp"

namespace py = pybind11;

namespace dart::python {

void py_so3(py::module& m)
{
  auto so3 = py::class_<math::SO3d>(m, "SO3")
                 .def(py::init<>())
                 .def("set_identity", &math::SO3d::set_identity)
                 .def(
                     "is_identity",
                     &math::SO3d::is_identity,
                     py::arg("tolerance") = math::eps<double>())
                 .def("set_random", &math::SO3d::set_random)
                 .def("inverse", &math::SO3d::inverse)
                 .def_static("Identity", &math::SO3d::Identity)
                 .def_static("Random", &math::SO3d::Random);

  auto so3_algebra = py::class_<math::SO3Algebrad>(m, "SO3Algebra")
                         .def(py::init<>())
                         .def_static("Zero", &math::SO3Algebrad::Zero)
                         .def_static("Random", &math::SO3Algebrad::Random);

  auto so3_tangent = py::class_<math::SO3Tangentd>(m, "SO3Tangent")
                         .def(py::init<>())
                         .def(
                             "is_zero",
                             &math::SO3Tangentd::is_zero,
                             py::arg("tolerance") = math::eps<double>())
                         .def_static("Zero", &math::SO3Tangentd::Zero)
                         .def_static("Random", &math::SO3Tangentd::Random);

  auto so3_cotangent = py::class_<math::SO3Cotangentd>(m, "SO3Cotangent")
                           .def(py::init<>())
                           .def(
                               "is_zero",
                               &math::SO3Cotangentd::is_zero,
                               py::arg("tolerance") = math::eps<double>())
                           .def_static("Zero", &math::SO3Cotangentd::Zero)
                           .def_static("Random", &math::SO3Cotangentd::Random);

  so3.attr("Algebra") = so3_algebra;
  so3.attr("Tangent") = so3_tangent;
  so3.attr("Cotangent") = so3_cotangent;
}

} // namespace dart::python
