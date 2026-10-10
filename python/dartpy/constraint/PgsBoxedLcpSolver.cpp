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
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <memory>
#include <string>

namespace dart {
namespace python {

void PgsBoxedLcpSolver(nb::module_& m)
{
  dartnb::dart_class<dart::constraint::PgsBoxedLcpSolver::Option>(
      m, "PgsBoxedLcpSolverOption")
      .def(dartnb::init<>())
      .def(dartnb::init<int>(), nb::arg("maxIteration"))
      .def(
          dartnb::init<int, double>(),
          nb::arg("maxIteration"),
          nb::arg("deltaXTolerance"))
      .def(
          dartnb::init<int, double, double>(),
          nb::arg("maxIteration"),
          nb::arg("deltaXTolerance"),
          nb::arg("relativeDeltaXTolerance"))
      .def(
          dartnb::init<int, double, double, double>(),
          nb::arg("maxIteration"),
          nb::arg("deltaXTolerance"),
          nb::arg("relativeDeltaXTolerance"),
          nb::arg("epsilonForDivision"))
      .def(
          dartnb::init<int, double, double, double, bool>(),
          nb::arg("maxIteration"),
          nb::arg("deltaXTolerance"),
          nb::arg("relativeDeltaXTolerance"),
          nb::arg("epsilonForDivision"),
          nb::arg("randomizeConstraintOrder"))
      .def_rw(
          "mMaxIteration",
          &dart::constraint::PgsBoxedLcpSolver::Option::mMaxIteration,
          dartnb::setterArgument(
              &dart::constraint::PgsBoxedLcpSolver::Option::mMaxIteration))
      .def_rw(
          "mDeltaXThreshold",
          &dart::constraint::PgsBoxedLcpSolver::Option::mDeltaXThreshold,
          dartnb::setterArgument(
              &dart::constraint::PgsBoxedLcpSolver::Option::mDeltaXThreshold))
      .def_rw(
          "mRelativeDeltaXTolerance",
          &dart::constraint::PgsBoxedLcpSolver::Option::
              mRelativeDeltaXTolerance,
          dartnb::setterArgument(&dart::constraint::PgsBoxedLcpSolver::Option::
                                     mRelativeDeltaXTolerance))
      .def_rw(
          "mEpsilonForDivision",
          &dart::constraint::PgsBoxedLcpSolver::Option::mEpsilonForDivision,
          dartnb::setterArgument(&dart::constraint::PgsBoxedLcpSolver::Option::
                                     mEpsilonForDivision))
      .def_rw(
          "mRandomizeConstraintOrder",
          &dart::constraint::PgsBoxedLcpSolver::Option::
              mRandomizeConstraintOrder,
          dartnb::setterArgument(&dart::constraint::PgsBoxedLcpSolver::Option::
                                     mRandomizeConstraintOrder));

  dartnb::dart_class<
      dart::constraint::PgsBoxedLcpSolver,
      dart::constraint::BoxedLcpSolver>(m, "PgsBoxedLcpSolver")
      .def(
          "getType",
          +[](const dart::constraint::PgsBoxedLcpSolver* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "solve",
          +[](dart::constraint::PgsBoxedLcpSolver* self,
              int n,
              double* A,
              double* x,
              double* b,
              int nub,
              double* lo,
              double* hi,
              int* findex,
              bool earlyTermination) -> bool {
            return self->solve(
                n, A, x, b, nub, lo, hi, findex, earlyTermination);
          },
          nb::arg("n"),
          nb::arg("A").none(),
          nb::arg("x").none(),
          nb::arg("b").none(),
          nb::arg("nub"),
          nb::arg("lo").none(),
          nb::arg("hi").none(),
          nb::arg("findex").none(),
          nb::arg("earlyTermination"))
      .def(
          "setOption",
          +[](dart::constraint::PgsBoxedLcpSolver* self,
              const dart::constraint::PgsBoxedLcpSolver::Option& option) {
            self->setOption(option);
          },
          nb::arg("option"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::constraint::PgsBoxedLcpSolver::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart
