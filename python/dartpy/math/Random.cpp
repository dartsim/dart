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

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void Random(py::module& m)
{
  ::py::class_<dart::math::Random<>>(m, "Random")
      .def(::py::init<uint32_t>(), ::py::arg("seed") = std::random_device{}())
      .def("setSeed", &dart::math::Random<>::setSeed, ::py::arg("seed"))
      .def("getSeed", &dart::math::Random<>::getSeed)
      .def(
          "uniformInt",
          +[](dart::math::Random<>& self, int min, int max) {
            return self.uniform<int>(min, max);
          },
          ::py::arg("min"),
          ::py::arg("max"));

  m.def(
      "UniformInt",
      &dart::math::Uniform<int>,
      ::py::arg("min"),
      ::py::arg("max"));

  m.def(
      "UniformInt",
      +[](int n, int min, int max) {
        return dart::math::Uniform<math::VectorXi>(n, min, max);
      },
      ::py::arg("n"),
      ::py::arg("min"),
      ::py::arg("max"));

  m.def(
      "UniformFloat",
      &dart::math::Uniform<double>,
      ::py::arg("min"),
      ::py::arg("max"));

  m.def(
      "UniformFloat",
      +[](int n, double min, double max) {
        return dart::math::Uniform<math::VectorXd>(n, min, max);
      },
      ::py::arg("n"),
      ::py::arg("min"),
      ::py::arg("max"));
}

} // namespace python
} // namespace dart
