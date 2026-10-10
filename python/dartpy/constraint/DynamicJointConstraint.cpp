// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

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

#include <dart/constraint/BallJointConstraint.hpp>
#include <dart/constraint/ConstraintBase.hpp>
#include <dart/constraint/CylindricalJointConstraint.hpp>
#include <dart/constraint/DynamicJointConstraint.hpp>
#include <dart/constraint/RevoluteJointConstraint.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>

#include <dart/dynamics/BodyNode.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart {
namespace python {

void DynamicJointConstraint(nb::module_& m)
{
  dartnb::dart_class<
      dart::constraint::DynamicJointConstraint,
      dart::constraint::ConstraintBase>(m, "DynamicJointConstraint")
      .def_static(
          "setErrorAllowance",
          +[](double allowance) {
            dart::constraint::DynamicJointConstraint::setErrorAllowance(
                allowance);
          },
          nb::arg("allowance"))
      .def_static(
          "getErrorAllowance",
          +[]() -> double {
            return dart::constraint::DynamicJointConstraint::
                getErrorAllowance();
          })
      .def_static(
          "setErrorReductionParameter",
          +[](double erp) {
            dart::constraint::DynamicJointConstraint::
                setErrorReductionParameter(erp);
          },
          nb::arg("erp"))
      .def_static(
          "getErrorReductionParameter",
          +[]() -> double {
            return dart::constraint::DynamicJointConstraint::
                getErrorReductionParameter();
          })
      .def_static(
          "setMaxErrorReductionVelocity",
          +[](double erv) {
            dart::constraint::DynamicJointConstraint::
                setMaxErrorReductionVelocity(erv);
          },
          nb::arg("erv"))
      .def_static(
          "getMaxErrorReductionVelocity",
          +[]() -> double {
            return dart::constraint::DynamicJointConstraint::
                getMaxErrorReductionVelocity();
          })
      .def_static(
          "setConstraintForceMixing",
          +[](double cfm) {
            dart::constraint::DynamicJointConstraint::setConstraintForceMixing(
                cfm);
          },
          nb::arg("cfm"))
      .def_static(
          "getConstraintForceMixing", +[]() -> double {
            return dart::constraint::DynamicJointConstraint::
                getConstraintForceMixing();
          });

  dartnb::dart_class<
      dart::constraint::BallJointConstraint,
      dart::constraint::DynamicJointConstraint>(m, "BallJointConstraint")
      .def(
          dartnb::init<dart::dynamics::BodyNode*, const Eigen::Vector3d&>(),
          nb::arg("body").none(),
          nb::arg("jointPos"))
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&>(),
          nb::arg("body1").none(),
          nb::arg("body2").none(),
          nb::arg("jointPos"))
      .def_static(
          "getStaticType", +[]() -> std::string {
            return dart::constraint::BallJointConstraint::getStaticType();
          });

  dartnb::dart_class<
      dart::constraint::CylindricalJointConstraint,
      dart::constraint::DynamicJointConstraint>(m, "CylindricalJointConstraint")
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("body").none(),
          nb::arg("jointPos"),
          nb::arg("axis"))
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("body1").none(),
          nb::arg("body2").none(),
          nb::arg("jointPos"),
          nb::arg("axis1"),
          nb::arg("axis2"))
      .def_static(
          "getStaticType", +[]() -> std::string {
            return dart::constraint::CylindricalJointConstraint::
                getStaticType();
          });

  dartnb::dart_class<
      dart::constraint::RevoluteJointConstraint,
      dart::constraint::DynamicJointConstraint>(m, "RevoluteJointConstraint")
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("body").none(),
          nb::arg("jointPos"),
          nb::arg("axis"))
      .def(
          dartnb::init<
              dart::dynamics::BodyNode*,
              dart::dynamics::BodyNode*,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::arg("body1").none(),
          nb::arg("body2").none(),
          nb::arg("jointPos"),
          nb::arg("axis1"),
          nb::arg("axis2"))
      .def_static(
          "getStaticType", +[]() -> std::string {
            return dart::constraint::RevoluteJointConstraint::getStaticType();
          });

  dartnb::dart_class<
      dart::constraint::WeldJointConstraint,
      dart::constraint::DynamicJointConstraint>(m, "WeldJointConstraint")
      .def(dartnb::init<dart::dynamics::BodyNode*>(), nb::arg("body").none())
      .def(
          dartnb::init<dart::dynamics::BodyNode*, dart::dynamics::BodyNode*>(),
          nb::arg("body1").none(),
          nb::arg("body2").none())
      .def_static(
          "getStaticType",
          +[]() -> std::string {
            return dart::constraint::WeldJointConstraint::getStaticType();
          })
      .def(
          "setRelativeTransform",
          +[](dart::constraint::WeldJointConstraint* self,
              const Eigen::Isometry3d& tf) { self->setRelativeTransform(tf); },
          nb::arg("tf"));
}

} // namespace python
} // namespace dart
