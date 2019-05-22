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

void JointLimitConstraint(pybind11::module& m)
{
  ::pybind11::class_<
      dart::constraint::JointLimitConstraint,
      dart::constraint::ConstraintBase,
      std::shared_ptr<dart::constraint::JointLimitConstraint>>(
      m, "JointLimitConstraint")
      .def(::pybind11::init<dart::dynamics::Joint*>(), ::pybind11::arg("joint"))
      .def_static(
          "setErrorAllowance",
          +[](double _allowance) {
            dart::constraint::JointLimitConstraint::setErrorAllowance(
                _allowance);
          },
          ::pybind11::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::constraint::JointLimitConstraint::getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double _erp) {
            dart::constraint::JointLimitConstraint::setErrorReductionParameter(
                _erp);
          },
          ::pybind11::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::constraint::JointLimitConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double _erv) {
            dart::constraint::JointLimitConstraint::
                setMaxErrorReductionVelocity(_erv);
          },
          ::pybind11::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          +[]() -> double {
            return dart::constraint::JointLimitConstraint::
                getMaxErrorReductionVelocity();
          })
      .def_static(
          "setConstraintForceMixing",
          +[](double _cfm) {
            dart::constraint::JointLimitConstraint::setConstraintForceMixing(
                _cfm);
          },
          ::pybind11::arg("cfm"))
      .def_static("getConstraintForceMixing", +[]() -> double {
        return dart::constraint::JointLimitConstraint::
            getConstraintForceMixing();
      });
}

} // namespace python
} // namespace dart
