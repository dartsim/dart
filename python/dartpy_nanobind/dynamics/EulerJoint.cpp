// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/unique_ptr.h>

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

#include "Joint.hpp"

#include <dart/dynamics/EulerJoint.hpp>
#include <dart/dynamics/GenericJoint.hpp>

#include <dart/math/ConfigurationSpace.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/EmbeddedAspect.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_geometry_pybind.h>

#include <memory>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

void EulerJoint(nb::module_& m)
{
  dartnb::dart_class<dart::dynamics::EulerJoint::UniqueProperties>(
      m, "EulerJointUniqueProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<dart::dynamics::detail::AxisOrder>(),
          nb::arg("axisOrder"));

  dartnb::dart_class<
      dart::dynamics::EulerJoint::Properties,
      dart::dynamics::GenericJoint<math::R3Space>::Properties,
      dart::dynamics::EulerJoint::UniqueProperties>(m, "EulerJointProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<const dart::dynamics::GenericJoint<
              dart::math::R3Space>::Properties&>(),
          nb::arg("genericJointProperties"))
      .def(
          dartnb::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R3Space>::Properties&,
              const dart::dynamics::EulerJoint::UniqueProperties&>(),
          nb::arg("genericJointProperties"),
          nb::arg("uniqueProperties"))
      .def_rw(
          "mAxisOrder",
          &dart::dynamics::detail::EulerJointUniqueProperties::mAxisOrder,
          dartnb::setterArgument(
              &dart::dynamics::detail::EulerJointUniqueProperties::mAxisOrder));

  DARTPY_DEFINE_JOINT_COMMON_BASE(EulerJoint, R3Space)

  dartnb::dart_class<
      dart::dynamics::EulerJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::EulerJoint,
          dart::dynamics::detail::EulerJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>>>(
      m, "EulerJoint")
      .def(
          "hasEulerJointAspect",
          +[](const dart::dynamics::EulerJoint* self) -> bool {
            return self->hasEulerJointAspect();
          })
      .def(
          "setEulerJointAspect",
          +[](dart::dynamics::EulerJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::EulerJoint,
                  dart::dynamics::detail::EulerJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::Aspect* aspect) {
            self->setEulerJointAspect(aspect);
          },
          nb::arg("aspect").none())
      .def(
          "removeEulerJointAspect",
          +[](dart::dynamics::EulerJoint* self) {
            self->removeEulerJointAspect();
          })
      .def(
          "releaseEulerJointAspect",
          +[](dart::dynamics::EulerJoint* self)
              -> std::unique_ptr<dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::EulerJoint,
                  dart::dynamics::detail::EulerJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::Aspect> {
            return self->releaseEulerJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::EulerJoint* self,
              const dart::dynamics::EulerJoint::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::EulerJoint* self,
              const dart::dynamics::EulerJoint::UniqueProperties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::EulerJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::EulerJoint,
                  dart::dynamics::detail::EulerJointUniqueProperties,
                  dart::dynamics::GenericJoint<
                      dart::math::RealVectorSpace<3>>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          nb::arg("properties"))
      .def(
          "getEulerJointProperties",
          +[](const dart::dynamics::EulerJoint* self)
              -> dart::dynamics::EulerJoint::Properties {
            return self->getEulerJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::EulerJoint* self,
              const dart::dynamics::EulerJoint* _otherJoint) {
            self->copy(_otherJoint);
          },
          nb::arg("otherJoint").none())
      .def(
          "getType",
          +[](const dart::dynamics::EulerJoint* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::EulerJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          nb::arg("index"))
      .def(
          "setAxisOrder",
          +[](dart::dynamics::EulerJoint* self,
              dart::dynamics::EulerJoint::AxisOrder _order) {
            self->setAxisOrder(_order);
          },
          nb::arg("order"))
      .def(
          "setAxisOrder",
          +[](dart::dynamics::EulerJoint* self,
              dart::dynamics::EulerJoint::AxisOrder _order,
              bool _renameDofs) { self->setAxisOrder(_order, _renameDofs); },
          nb::arg("order"),
          nb::arg("renameDofs"))
      .def(
          "getAxisOrder",
          +[](const dart::dynamics::EulerJoint* self)
              -> dart::dynamics::EulerJoint::AxisOrder {
            return self->getAxisOrder();
          })
      .def(
          "convertToTransform",
          +[](const dart::dynamics::EulerJoint* self,
              const Eigen::Vector3d& _positions) -> Eigen::Isometry3d {
            return self->convertToTransform(_positions);
          },
          nb::arg("positions"))
      .def(
          "convertToRotation",
          +[](const dart::dynamics::EulerJoint* self,
              const Eigen::Vector3d& _positions) -> Eigen::Matrix3d {
            return self->convertToRotation(_positions);
          },
          nb::arg("positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::EulerJoint* self,
              const Eigen::Vector3d& _positions)
              -> Eigen::Matrix<double, 6, 3> {
            return self->getRelativeJacobianStatic(_positions);
          },
          nb::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::EulerJoint::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "convertToTransformOf",
          +[](const Eigen::Vector3d& _positions,
              dart::dynamics::EulerJoint::AxisOrder _ordering)
              -> Eigen::Isometry3d {
            return dart::dynamics::EulerJoint::convertToTransform(
                _positions, _ordering);
          },
          nb::arg("positions"),
          nb::arg("ordering"))
      .def_static(
          "convertToRotationOf",
          +[](const Eigen::Vector3d& _positions,
              dart::dynamics::EulerJoint::AxisOrder _ordering)
              -> Eigen::Matrix3d {
            return dart::dynamics::EulerJoint::convertToRotation(
                _positions, _ordering);
          },
          nb::arg("positions"),
          nb::arg("ordering"));
}

} // namespace python
} // namespace dart
