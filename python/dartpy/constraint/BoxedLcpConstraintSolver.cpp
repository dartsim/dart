/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void BoxedLcpConstraintSolver(py::module& m)
{
  py::enum_<constraint::BoxedLcpSolverType>(m, "BoxedLcpSolverType")
      .value(
          "Dantzig",
          constraint::BoxedLcpSolverType::Dantzig,
          "The Dantzig solver")
      .value(
          "Pgs",
          constraint::BoxedLcpSolverType::Pgs,
          "The projected Gauss-Seidel solver")
      .export_values();

  // Bind the struct BoxedLcpConstraintSolverConfig
  py::class_<constraint::BoxedLcpConstraintSolverConfig>(
      m, "BoxedLcpConstraintSolverConfig")
      .def(py::init<>())
      .def_readwrite(
          "primaryBoxedLcpSolver",
          &constraint::BoxedLcpConstraintSolverConfig::primaryBoxedLcpSolver)
      .def_readwrite(
          "secondaryBoxedLcpSolver",
          &constraint::BoxedLcpConstraintSolverConfig::secondaryBoxedLcpSolver);

  ::py::class_<
      constraint::BoxedLcpConstraintSolver,
      constraint::ConstraintSolver,
      std::shared_ptr<constraint::BoxedLcpConstraintSolver>>(
      m, "BoxedLcpConstraintSolver")
      .def(
          py::init<const constraint::BoxedLcpConstraintSolverConfig&>(),
          py::arg("config") = constraint::BoxedLcpConstraintSolverConfig())
      .def(
          "setPrimaryBoxedLcpSolverType",
          &constraint::BoxedLcpConstraintSolver::setPrimaryBoxedLcpSolverType,
          ::py::arg("type"))
      .def(
          "getPrimaryBoxedLcpSolverType",
          &constraint::BoxedLcpConstraintSolver::getPrimaryBoxedLcpSolverType)
      .def(
          "setSecondaryBoxedLcpSolverType",
          &constraint::BoxedLcpConstraintSolver::setSecondaryBoxedLcpSolverType,
          ::py::arg("type"))
      .def(
          "getSecondaryBoxedLcpSolverType",
          &constraint::BoxedLcpConstraintSolver::
              getSecondaryBoxedLcpSolverType);
}

} // namespace python
} // namespace dart
