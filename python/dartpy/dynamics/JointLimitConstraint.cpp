/*
 * Copyright (c) 2011-2021, The DART development contributors
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

void JointLimitConstraint(py::module& m)
{
  ::py::class_<
      dart::dynamics::JointLimitConstraint,
      dart::dynamics::ConstraintBase,
      std::shared_ptr<dart::dynamics::JointLimitConstraint>>(
      m, "JointLimitConstraint")
      .def(::py::init<dart::dynamics::Joint*>(), ::py::arg("joint"))
      .def_static(
          "setErrorAllowance",
          +[](double _allowance) {
            dart::dynamics::JointLimitConstraint::setErrorAllowance(_allowance);
          },
          ::py::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::dynamics::JointLimitConstraint::getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double _erp) {
            dart::dynamics::JointLimitConstraint::setErrorReductionParameter(
                _erp);
          },
          ::py::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::dynamics::JointLimitConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double _erv) {
            dart::dynamics::JointLimitConstraint::setMaxErrorReductionVelocity(
                _erv);
          },
          ::py::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          +[]() -> double {
            return dart::dynamics::JointLimitConstraint::
                getMaxErrorReductionVelocity();
          })
      .def_static(
          "setConstraintForceMixing",
          +[](double _cfm) {
            dart::dynamics::JointLimitConstraint::setConstraintForceMixing(
                _cfm);
          },
          ::py::arg("cfm"))
      .def_static("getConstraintForceMixing", +[]() -> double {
        return dart::dynamics::JointLimitConstraint::getConstraintForceMixing();
      });
}

} // namespace python
} // namespace dart
