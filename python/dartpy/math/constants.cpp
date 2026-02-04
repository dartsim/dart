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

#include "math/constants.hpp"

#include "dart/math/constants.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defMathConstants(nb::module_& m)
{
  // Expose the same names as dart::math for quick parity checks.
  m.attr("pi") = dart::math::pi;
  m.attr("two_pi") = dart::math::two_pi;
  m.attr("half_pi") = dart::math::half_pi;
  m.attr("pi_sq") = dart::math::pi_sq;
  m.attr("phi") = dart::math::phi;
  m.attr("inf") = dart::math::inf;
  m.attr("max_val") = dart::math::max_val;
  m.attr("min_val") = dart::math::min_val;
  m.attr("eps") = dart::math::eps;

  m.def(
      "deg2rad",
      [](double degrees) { return degrees * dart::math::pi / 180.0; },
      nb::arg("degrees"),
      "Convert degrees to radians using high-precision constants.");

  m.def(
      "rad2deg",
      [](double radians) { return radians * 180.0 / dart::math::pi; },
      nb::arg("radians"),
      "Convert radians to degrees using high-precision constants.");
}

} // namespace dart::python_nb
