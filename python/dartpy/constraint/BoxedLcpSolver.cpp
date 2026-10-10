// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/constraint/BoxedLcpSolver.hpp>

#include <memory>
#include <string>

namespace dart {
namespace python {

void BoxedLcpSolver(nb::module_& m)
{
  dartnb::dart_class<dart::constraint::BoxedLcpSolver>(m, "BoxedLcpSolver")
      .def(
          "getType",
          +[](const dart::constraint::BoxedLcpSolver* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "solve",
          +[](dart::constraint::BoxedLcpSolver* self,
              int n,
              double* A,
              double* x,
              double* b,
              int nub,
              double* lo,
              double* hi,
              int* findex) { self->solve(n, A, x, b, nub, lo, hi, findex); },
          nb::arg("n"),
          nb::arg("A").none(),
          nb::arg("x").none(),
          nb::arg("b").none(),
          nb::arg("nub"),
          nb::arg("lo").none(),
          nb::arg("hi").none(),
          nb::arg("findex").none());
}

} // namespace python
} // namespace dart
