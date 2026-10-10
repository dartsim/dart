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

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <memory>

namespace dart {
namespace python {

void BoxedLcpConstraintSolver(nb::module_& m)
{
  using MatrixFreeContactSolverOptions
      = constraint::BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions;

  dartnb::dart_class<MatrixFreeContactSolverOptions>(
      m, "MatrixFreeContactSolverOptions")
      .def(dartnb::init<>())
      .def_rw(
          "mEnabled",
          &MatrixFreeContactSolverOptions::mEnabled,
          dartnb::setterArgument(&MatrixFreeContactSolverOptions::mEnabled))
      .def_rw(
          "mMinRows",
          &MatrixFreeContactSolverOptions::mMinRows,
          dartnb::setterArgument(&MatrixFreeContactSolverOptions::mMinRows))
      .def_rw(
          "mMaxIterations",
          &MatrixFreeContactSolverOptions::mMaxIterations,
          dartnb::setterArgument(
              &MatrixFreeContactSolverOptions::mMaxIterations))
      .def_rw(
          "mSor",
          &MatrixFreeContactSolverOptions::mSor,
          dartnb::setterArgument(&MatrixFreeContactSolverOptions::mSor))
      .def_rw(
          "mDeltaTolerance",
          &MatrixFreeContactSolverOptions::mDeltaTolerance,
          dartnb::setterArgument(
              &MatrixFreeContactSolverOptions::mDeltaTolerance))
      .def_rw(
          "mRelativeDeltaTolerance",
          &MatrixFreeContactSolverOptions::mRelativeDeltaTolerance,
          dartnb::setterArgument(
              &MatrixFreeContactSolverOptions::mRelativeDeltaTolerance))
      .def_rw(
          "mEpsilonForDivision",
          &MatrixFreeContactSolverOptions::mEpsilonForDivision,
          dartnb::setterArgument(
              &MatrixFreeContactSolverOptions::mEpsilonForDivision));

  dartnb::dart_class<
      constraint::BoxedLcpConstraintSolver,
      constraint::ConstraintSolver>(m, "BoxedLcpConstraintSolver")
      .def(dartnb::init<>())
      .def(
          dartnb::init<constraint::BoxedLcpSolverPtr>(),
          nb::arg("boxedLcpSolver").none())
      .def(
          dartnb::init<
              constraint::BoxedLcpSolverPtr,
              constraint::BoxedLcpSolverPtr>(),
          nb::arg("boxedLcpSolver").none(),
          nb::arg("secondaryBoxedLcpSolver").none())
      .def(
          "setBoxedLcpSolver",
          +[](constraint::BoxedLcpConstraintSolver* self,
              constraint::BoxedLcpSolverPtr lcpSolver) {
            self->setBoxedLcpSolver(lcpSolver);
          },
          nb::arg("lcpSolver").none())
      .def(
          "getBoxedLcpSolver",
          +[](const constraint::BoxedLcpConstraintSolver* self)
              -> constraint::ConstBoxedLcpSolverPtr {
            return self->getBoxedLcpSolver();
          })
      .def(
          "setSecondaryBoxedLcpSolver",
          +[](constraint::BoxedLcpConstraintSolver* self,
              constraint::BoxedLcpSolverPtr lcpSolver) {
            self->setSecondaryBoxedLcpSolver(lcpSolver);
          },
          nb::arg("lcpSolver").none())
      .def(
          "getSecondaryBoxedLcpSolver",
          +[](const constraint::BoxedLcpConstraintSolver* self)
              -> constraint::ConstBoxedLcpSolverPtr {
            return self->getSecondaryBoxedLcpSolver();
          })
      .def(
          "setMatrixFreeContactSolverOptions",
          +[](constraint::BoxedLcpConstraintSolver* self,
              const MatrixFreeContactSolverOptions& options) {
            self->setMatrixFreeContactSolverOptions(options);
          },
          nb::arg("options"))
      .def(
          "getMatrixFreeContactSolverOptions",
          +[](const constraint::BoxedLcpConstraintSolver* self)
              -> MatrixFreeContactSolverOptions {
            return self->getMatrixFreeContactSolverOptions();
          });
}

} // namespace python
} // namespace dart
