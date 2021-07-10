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
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "Joint.hpp"

namespace py = pybind11;

namespace dart {
namespace python {

void EulerJoint(py::module& m) {
  ::py::class_<dart::dynamics::EulerJoint::UniqueProperties>(
      m, "EulerJointUniqueProperties")
      .def(::py::init<>())
      .def(
          ::py::init<dart::dynamics::detail::AxisOrder>(),
          ::py::arg("axisOrder"));

  ::py::class_<
      dart::dynamics::EulerJoint::Properties,
      dart::dynamics::GenericJoint<math::R3Space>::Properties,
      dart::dynamics::EulerJoint::UniqueProperties>(m, "EulerJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::R3Space>::Properties&>(),
          ::py::arg("genericJointProperties"))
      .def(
          ::py::init<
              const dart::dynamics::GenericJoint<
                  dart::math::R3Space>::Properties&,
              const dart::dynamics::EulerJoint::UniqueProperties&>(),
          ::py::arg("genericJointProperties"),
          ::py::arg("uniqueProperties"))
      .def_readwrite(
          "mAxisOrder",
          &dart::dynamics::detail::EulerJointUniqueProperties::mAxisOrder);

  DARTPY_DEFINE_JOINT_COMMON_BASE(EulerJoint, R3Space)

  ::py::class_<
      dart::dynamics::EulerJoint,
      dart::common::EmbedPropertiesOnTopOf<
          dart::dynamics::EulerJoint,
          dart::dynamics::detail::EulerJointUniqueProperties,
          dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>>,
      std::shared_ptr<dart::dynamics::EulerJoint>>(m, "EulerJoint")
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
          ::py::arg("aspect"))
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
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::EulerJoint* self,
              const dart::dynamics::EulerJoint::UniqueProperties& _properties) {
            self->setProperties(_properties);
          },
          ::py::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::EulerJoint* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::EulerJoint,
                  dart::dynamics::detail::EulerJointUniqueProperties,
                  dart::dynamics::GenericJoint<dart::math::RealVectorSpace<
                      3>>>::AspectProperties& properties) {
            self->setAspectProperties(properties);
          },
          ::py::arg("properties"))
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
          ::py::arg("otherJoint"))
      .def(
          "getType",
          +[](const dart::dynamics::EulerJoint* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::EulerJoint* self,
              std::size_t _index) -> bool {
            return self->isCyclic(_index);
          },
          ::py::arg("index"))
      .def(
          "setAxisOrder",
          +[](dart::dynamics::EulerJoint* self,
              dart::dynamics::EulerJoint::AxisOrder _order) {
            self->setAxisOrder(_order);
          },
          ::py::arg("order"))
      .def(
          "setAxisOrder",
          +[](dart::dynamics::EulerJoint* self,
              dart::dynamics::EulerJoint::AxisOrder _order,
              bool _renameDofs) {
            self->setAxisOrder(_order, _renameDofs);
          },
          ::py::arg("order"),
          ::py::arg("renameDofs"))
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
          ::py::arg("positions"))
      .def(
          "convertToRotation",
          +[](const dart::dynamics::EulerJoint* self,
              const Eigen::Vector3d& _positions) -> Eigen::Matrix3d {
            return self->convertToRotation(_positions);
          },
          ::py::arg("positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::EulerJoint* self,
              const Eigen::Vector3d& _positions)
              -> Eigen::Matrix<double, 6, 3> {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::py::arg("positions"))
      .def_static(
          "getStaticType",
          +[]() -> const std::
                    string& {
                      return dart::dynamics::EulerJoint::getStaticType();
                    },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "convertToTransformOf",
          +[](const Eigen::Vector3d& _positions,
              dart::dynamics::EulerJoint::AxisOrder _ordering)
              -> Eigen::Isometry3d {
            return dart::dynamics::EulerJoint::convertToTransform(
                _positions, _ordering);
          },
          ::py::arg("positions"),
          ::py::arg("ordering"))
      .def_static(
          "convertToRotationOf",
          +[](const Eigen::Vector3d& _positions,
              dart::dynamics::EulerJoint::AxisOrder _ordering)
              -> Eigen::Matrix3d {
            return dart::dynamics::EulerJoint::convertToRotation(
                _positions, _ordering);
          },
          ::py::arg("positions"),
          ::py::arg("ordering"));
}

} // namespace python
} // namespace dart
