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

#include <dart/constraint/ConstraintBase.hpp>
#include <dart/constraint/JointConstraint.hpp>

#include <dart/dynamics/Joint.hpp>

#include <memory>

namespace dart {
namespace python {

void JointConstraint(nb::module_& m)
{
  dartnb::dart_class<
      dart::constraint::JointConstraint,
      dart::constraint::ConstraintBase>(m, "JointConstraint")
      .def(dartnb::init<dart::dynamics::Joint*>(), nb::arg("joint").none())
      .def_static(
          "setErrorAllowance",
          +[](double allowance) {
            dart::constraint::JointConstraint::setErrorAllowance(allowance);
          },
          nb::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::constraint::JointConstraint::getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double _erp) {
            dart::constraint::JointConstraint::setErrorReductionParameter(_erp);
          },
          nb::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::constraint::JointConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double erv) {
            dart::constraint::JointConstraint::setMaxErrorReductionVelocity(
                erv);
          },
          nb::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          +[]() -> double {
            return dart::constraint::JointConstraint::
                getMaxErrorReductionVelocity();
          })
      .def_static(
          "setConstraintForceMixing",
          +[](double cfm) {
            dart::constraint::JointConstraint::setConstraintForceMixing(cfm);
          },
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing", +[]() -> double {
            return dart::constraint::JointConstraint::
                getConstraintForceMixing();
          });
}

} // namespace python
} // namespace dart
