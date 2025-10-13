/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/constraint/BalanceConstraint.hpp>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void BalanceConstraint(py::module& m)
{
  auto bc = ::py::class_<
      dart::constraint::BalanceConstraint,
      dart::optimizer::Function,
      std::shared_ptr<dart::constraint::BalanceConstraint>>(
      m, "BalanceConstraint");

  // Bind ErrorMethod_t enum BEFORE constructor
  ::py::enum_<dart::constraint::BalanceConstraint::ErrorMethod_t>(
      bc, "ErrorMethod")
      .value(
          "FROM_CENTROID", dart::constraint::BalanceConstraint::FROM_CENTROID)
      .value("FROM_EDGE", dart::constraint::BalanceConstraint::FROM_EDGE)
      .value(
          "OPTIMIZE_BALANCE",
          dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE)
      .export_values();

  // Bind BalanceMethod_t enum BEFORE constructor
  ::py::enum_<dart::constraint::BalanceConstraint::BalanceMethod_t>(
      bc, "BalanceMethod")
      .value(
          "SHIFT_SUPPORT", dart::constraint::BalanceConstraint::SHIFT_SUPPORT)
      .value("SHIFT_COM", dart::constraint::BalanceConstraint::SHIFT_COM)
      .export_values();

  // Now bind constructor with default enum values
  bc.def(
        ::py::init<
            const std::shared_ptr<dart::dynamics::HierarchicalIK>&,
            dart::constraint::BalanceConstraint::BalanceMethod_t,
            dart::constraint::BalanceConstraint::ErrorMethod_t>(),
        ::py::arg("ik"),
        ::py::arg("balanceMethod")
        = dart::constraint::BalanceConstraint::SHIFT_SUPPORT,
        ::py::arg("errorMethod")
        = dart::constraint::BalanceConstraint::FROM_CENTROID)
      .def(
          "setErrorMethod",
          &dart::constraint::BalanceConstraint::setErrorMethod,
          ::py::arg("method"))
      .def(
          "getErrorMethod",
          &dart::constraint::BalanceConstraint::getErrorMethod)
      .def(
          "setBalanceMethod",
          &dart::constraint::BalanceConstraint::setBalanceMethod,
          ::py::arg("method"))
      .def(
          "getBalanceMethod",
          &dart::constraint::BalanceConstraint::getBalanceMethod)
      .def(
          "setOptimizationTolerance",
          &dart::constraint::BalanceConstraint::setOptimizationTolerance,
          ::py::arg("tol"))
      .def(
          "getOptimizationTolerance",
          &dart::constraint::BalanceConstraint::getOptimizationTolerance)
      .def(
          "setPseudoInverseDamping",
          &dart::constraint::BalanceConstraint::setPseudoInverseDamping,
          ::py::arg("damping"))
      .def(
          "getPseudoInverseDamping",
          &dart::constraint::BalanceConstraint::getPseudoInverseDamping)
      .def(
          "getLastError",
          &dart::constraint::BalanceConstraint::getLastError,
          ::py::return_value_policy::reference_internal)
      .def("clearCaches", &dart::constraint::BalanceConstraint::clearCaches);
}

} // namespace python
} // namespace dart
