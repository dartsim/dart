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

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void PgsBoxedLcpSolver(py::module& m)
{
  ::py::class_<dart::dynamics::PgsBoxedLcpSolver::Option>(
      m, "PgsBoxedLcpSolverOption")
      .def(::py::init<>())
      .def(::py::init<int>(), ::py::arg("maxIteration"))
      .def(
          ::py::init<int, double>(),
          ::py::arg("maxIteration"),
          ::py::arg("deltaXTolerance"))
      .def(
          ::py::init<int, double, double>(),
          ::py::arg("maxIteration"),
          ::py::arg("deltaXTolerance"),
          ::py::arg("relativeDeltaXTolerance"))
      .def(
          ::py::init<int, double, double, double>(),
          ::py::arg("maxIteration"),
          ::py::arg("deltaXTolerance"),
          ::py::arg("relativeDeltaXTolerance"),
          ::py::arg("epsilonForDivision"))
      .def(
          ::py::init<int, double, double, double, bool>(),
          ::py::arg("maxIteration"),
          ::py::arg("deltaXTolerance"),
          ::py::arg("relativeDeltaXTolerance"),
          ::py::arg("epsilonForDivision"),
          ::py::arg("randomizeConstraintOrder"))
      .def_readwrite(
          "mMaxIteration",
          &dart::dynamics::PgsBoxedLcpSolver::Option::mMaxIteration)
      .def_readwrite(
          "mDeltaXThreshold",
          &dart::dynamics::PgsBoxedLcpSolver::Option::mDeltaXThreshold)
      .def_readwrite(
          "mRelativeDeltaXTolerance",
          &dart::dynamics::PgsBoxedLcpSolver::Option::mRelativeDeltaXTolerance)
      .def_readwrite(
          "mEpsilonForDivision",
          &dart::dynamics::PgsBoxedLcpSolver::Option::mEpsilonForDivision)
      .def_readwrite(
          "mRandomizeConstraintOrder",
          &dart::dynamics::PgsBoxedLcpSolver::Option::
              mRandomizeConstraintOrder);

  ::py::class_<
      dart::dynamics::PgsBoxedLcpSolver,
      dart::dynamics::BoxedLcpSolver,
      std::shared_ptr<dart::dynamics::PgsBoxedLcpSolver>>(
      m, "PgsBoxedLcpSolver")
      .def(
          "getType",
          +[](const dart::dynamics::PgsBoxedLcpSolver* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "solve",
          +[](dart::dynamics::PgsBoxedLcpSolver* self,
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
          ::py::arg("n"),
          ::py::arg("A"),
          ::py::arg("x"),
          ::py::arg("b"),
          ::py::arg("nub"),
          ::py::arg("lo"),
          ::py::arg("hi"),
          ::py::arg("findex"),
          ::py::arg("earlyTermination"))
      .def(
          "setOption",
          +[](dart::dynamics::PgsBoxedLcpSolver* self,
              const dart::dynamics::PgsBoxedLcpSolver::Option& option) {
            self->setOption(option);
          },
          ::py::arg("option"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::PgsBoxedLcpSolver::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
