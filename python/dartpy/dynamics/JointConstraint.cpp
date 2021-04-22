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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void JointConstraint(py::module& m)
{
  ::py::class_<
      dart::dynamics::JointConstraint,
      dart::dynamics::ConstraintBase,
      std::shared_ptr<dart::dynamics::JointConstraint>>(m, "JointConstraint")
      .def_static(
          "setErrorAllowance",
          +[](double allowance) {
            dart::dynamics::JointConstraint::setErrorAllowance(allowance);
          },
          ::py::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::dynamics::JointConstraint::getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double erp) {
            dart::dynamics::JointConstraint::setErrorReductionParameter(erp);
          },
          ::py::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::dynamics::JointConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double _erv) {
            dart::dynamics::JointConstraint::setMaxErrorReductionVelocity(_erv);
          },
          ::py::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          +[]() -> double {
            return dart::dynamics::JointConstraint::
                getMaxErrorReductionVelocity();
          })
      .def_static(
          "setConstraintForceMixing",
          +[](double cfm) {
            dart::dynamics::JointConstraint::setConstraintForceMixing(cfm);
          },
          ::py::arg("cfm"))
      .def_static("getConstraintForceMixing", +[]() -> double {
        return dart::dynamics::JointConstraint::getConstraintForceMixing();
      });

  ::py::class_<
      dart::dynamics::BallJointConstraint,
      dart::dynamics::JointConstraint,
      std::shared_ptr<dart::dynamics::BallJointConstraint>>(
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
        return dart::dynamics::BallJointConstraint::getStaticType();
      });

  ::py::class_<
      dart::dynamics::WeldJointConstraint,
      dart::dynamics::JointConstraint,
      std::shared_ptr<dart::dynamics::WeldJointConstraint>>(
      m, "WeldJointConstraint")
      .def(::py::init<dart::dynamics::BodyNode*>(), ::py::arg("body"))
      .def(
          ::py::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          ::py::arg("body1"),
          ::py::arg("body2"))
      .def_static(
          "getStaticType",
          +[]() -> std::string {
            return dart::dynamics::WeldJointConstraint::getStaticType();
          })
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::WeldJointConstraint* self,
              const Eigen::Isometry3d& tf) { self->setRelativeTransform(tf); },
          ::py::arg("tf"));
}

} // namespace python
} // namespace dart
