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

namespace dart {
namespace python {

void PgsBoxedLcpSolver(pybind11::module& m)
{
  ::pybind11::class_<dart::constraint::PgsBoxedLcpSolver::Option>(
      m, "PgsBoxedLcpSolverOption")
      .def(::pybind11::init<>())
      .def(::pybind11::init<int>(), ::pybind11::arg("maxIteration"))
      .def(
          ::pybind11::init<int, double>(),
          ::pybind11::arg("maxIteration"),
          ::pybind11::arg("deltaXTolerance"))
      .def(
          ::pybind11::init<int, double, double>(),
          ::pybind11::arg("maxIteration"),
          ::pybind11::arg("deltaXTolerance"),
          ::pybind11::arg("relativeDeltaXTolerance"))
      .def(
          ::pybind11::init<int, double, double, double>(),
          ::pybind11::arg("maxIteration"),
          ::pybind11::arg("deltaXTolerance"),
          ::pybind11::arg("relativeDeltaXTolerance"),
          ::pybind11::arg("epsilonForDivision"))
      .def(
          ::pybind11::init<int, double, double, double, bool>(),
          ::pybind11::arg("maxIteration"),
          ::pybind11::arg("deltaXTolerance"),
          ::pybind11::arg("relativeDeltaXTolerance"),
          ::pybind11::arg("epsilonForDivision"),
          ::pybind11::arg("randomizeConstraintOrder"))
      .def_readwrite(
          "mMaxIteration",
          &dart::constraint::PgsBoxedLcpSolver::Option::mMaxIteration)
      .def_readwrite(
          "mDeltaXThreshold",
          &dart::constraint::PgsBoxedLcpSolver::Option::mDeltaXThreshold)
      .def_readwrite(
          "mRelativeDeltaXTolerance",
          &dart::constraint::PgsBoxedLcpSolver::Option::
              mRelativeDeltaXTolerance)
      .def_readwrite(
          "mEpsilonForDivision",
          &dart::constraint::PgsBoxedLcpSolver::Option::mEpsilonForDivision)
      .def_readwrite(
          "mRandomizeConstraintOrder",
          &dart::constraint::PgsBoxedLcpSolver::Option::
              mRandomizeConstraintOrder);

  ::pybind11::class_<
      dart::constraint::PgsBoxedLcpSolver,
      dart::constraint::BoxedLcpSolver,
      std::shared_ptr<dart::constraint::PgsBoxedLcpSolver>>(
      m, "PgsBoxedLcpSolver")
      .def(
          "getType",
          +[](const dart::constraint::PgsBoxedLcpSolver* self)
              -> const std::string& { return self->getType(); },
          ::pybind11::return_value_policy::reference_internal)
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
          ::pybind11::arg("n"),
          ::pybind11::arg("A"),
          ::pybind11::arg("x"),
          ::pybind11::arg("b"),
          ::pybind11::arg("nub"),
          ::pybind11::arg("lo"),
          ::pybind11::arg("hi"),
          ::pybind11::arg("findex"),
          ::pybind11::arg("earlyTermination"))
      .def(
          "setOption",
          +[](dart::constraint::PgsBoxedLcpSolver* self,
              const dart::constraint::PgsBoxedLcpSolver::Option& option) {
            self->setOption(option);
          },
          ::pybind11::arg("option"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::constraint::PgsBoxedLcpSolver::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
