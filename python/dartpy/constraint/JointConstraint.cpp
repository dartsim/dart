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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void JointConstraint(py::module& m)
{
  ::py::class_<
      dart::constraint::JointConstraint,
      dart::constraint::ConstraintBase,
      std::shared_ptr<dart::constraint::JointConstraint>>(m, "JointConstraint")
      .def_static(
          "setErrorAllowance",
          +[](double allowance) {
            dart::constraint::JointConstraint::setErrorAllowance(allowance);
          },
          ::py::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::constraint::JointConstraint::getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double erp) {
            dart::constraint::JointConstraint::setErrorReductionParameter(erp);
          },
          ::py::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::constraint::JointConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double _erv) {
            dart::constraint::JointConstraint::setMaxErrorReductionVelocity(
                _erv);
          },
          ::py::arg("erv"))
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
          ::py::arg("cfm"))
      .def_static("getConstraintForceMixing", +[]() -> double {
        return dart::constraint::JointConstraint::getConstraintForceMixing();
      });

  ::py::class_<
      dart::constraint::BallJointConstraint,
      dart::constraint::JointConstraint,
      std::shared_ptr<dart::constraint::BallJointConstraint>>(
      m, "BallJointConstraint")
      .def(
          ::py::init<dart::dynamics::BodyNode*, const Eigen::Vector3d&>(),
          ::py::arg("body"),
          ::py::arg("jointPos"))
      .def(
          ::py::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&>(),
          ::py::arg("body1"),
          ::py::arg("body2"),
          ::py::arg("jointPos"))
      .def_static("getStaticType", +[]() -> std::string {
        return dart::constraint::BallJointConstraint::getStaticType();
      });

  ::py::class_<
      dart::constraint::WeldJointConstraint,
      dart::constraint::JointConstraint,
      std::shared_ptr<dart::constraint::WeldJointConstraint>>(
      m, "WeldJointConstraint")
      .def(::py::init<dart::dynamics::BodyNode*>(), ::py::arg("body"))
      .def(
          ::py::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          ::py::arg("body1"),
          ::py::arg("body2"))
      .def_static(
          "getStaticType",
          +[]() -> std::string {
            return dart::constraint::WeldJointConstraint::getStaticType();
          })
      .def(
          "setRelativeTransform",
          +[](dart::constraint::WeldJointConstraint* self,
              const Eigen::Isometry3d& tf) { self->setRelativeTransform(tf); },
          ::py::arg("tf"));
}

} // namespace python
} // namespace dart
