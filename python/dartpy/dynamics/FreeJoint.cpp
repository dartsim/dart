/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void FreeJoint(py::module& m)
{
  ::py::class_<
      dart::dynamics::FreeJoint::Properties,
      dart::dynamics::GenericJoint<math::SE3Space>::Properties>(
      m, "FreeJointProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::dynamics::GenericJoint<
              dart::math::SE3Space>::Properties&>(),
          ::py::arg("properties"));

  ::py::class_<
      dart::dynamics::FreeJoint,
      dart::dynamics::GenericJoint<dart::math::SE3Space>,
      std::shared_ptr<dart::dynamics::FreeJoint>>(m, "FreeJoint")
      .def(
          "getFreeJointProperties",
          +[](const dart::dynamics::FreeJoint* self)
              -> dart::dynamics::FreeJoint::Properties {
            return self->getFreeJointProperties();
          })
      .def(
          "getType",
          +[](const dart::dynamics::FreeJoint* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::FreeJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::py::arg("index"))
      .def(
          "setSpatialMotion",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d* newTransform,
              const dart::dynamics::Frame* withRespectTo,
              const math::Vector6d* newSpatialVelocity,
              const dart::dynamics::Frame* velRelativeTo,
              const dart::dynamics::Frame* velInCoordinatesOf,
              const math::Vector6d* newSpatialAcceleration,
              const dart::dynamics::Frame* accRelativeTo,
              const dart::dynamics::Frame* accInCoordinatesOf) {
            self->setSpatialMotion(
                newTransform,
                withRespectTo,
                newSpatialVelocity,
                velRelativeTo,
                velInCoordinatesOf,
                newSpatialAcceleration,
                accRelativeTo,
                accInCoordinatesOf);
          },
          ::py::arg("newTransform"),
          ::py::arg("withRespectTo"),
          ::py::arg("newSpatialVelocity"),
          ::py::arg("velRelativeTo"),
          ::py::arg("velInCoordinatesOf"),
          ::py::arg("newSpatialAcceleration"),
          ::py::arg("accRelativeTo"),
          ::py::arg("accInCoordinatesOf"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setRelativeTransform(newTransform);
          },
          ::py::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setTransform(newTransform);
          },
          ::py::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform,
              const dart::dynamics::Frame* withRespectTo) {
            self->setTransform(newTransform, withRespectTo);
          },
          ::py::arg("newTransform"),
          ::py::arg("withRespectTo"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialVelocity) {
            self->setRelativeSpatialVelocity(newSpatialVelocity);
          },
          ::py::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                newSpatialVelocity, inCoordinatesOf);
          },
          ::py::arg("newSpatialVelocity"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialVelocity(
                newSpatialVelocity, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newSpatialVelocity"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity) {
            self->setLinearVelocity(newLinearVelocity);
          },
          ::py::arg("newLinearVelocity"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearVelocity(newLinearVelocity, relativeTo);
          },
          ::py::arg("newLinearVelocity"),
          ::py::arg("relativeTo"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearVelocity(
                newLinearVelocity, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newLinearVelocity"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity) {
            self->setAngularVelocity(newAngularVelocity);
          },
          ::py::arg("newAngularVelocity"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularVelocity(newAngularVelocity, relativeTo);
          },
          ::py::arg("newAngularVelocity"),
          ::py::arg("relativeTo"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularVelocity(
                newAngularVelocity, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newAngularVelocity"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(newSpatialAcceleration);
          },
          ::py::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                newSpatialAcceleration, inCoordinatesOf);
          },
          ::py::arg("newSpatialAcceleration"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const math::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialAcceleration(
                newSpatialAcceleration, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newSpatialAcceleration"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration) {
            self->setLinearAcceleration(newLinearAcceleration);
          },
          ::py::arg("newLinearAcceleration"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearAcceleration(newLinearAcceleration, relativeTo);
          },
          ::py::arg("newLinearAcceleration"),
          ::py::arg("relativeTo"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearAcceleration(
                newLinearAcceleration, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newLinearAcceleration"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration) {
            self->setAngularAcceleration(newAngularAcceleration);
          },
          ::py::arg("newAngularAcceleration"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularAcceleration(newAngularAcceleration, relativeTo);
          },
          ::py::arg("newAngularAcceleration"),
          ::py::arg("relativeTo"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularAcceleration(
                newAngularAcceleration, relativeTo, inCoordinatesOf);
          },
          ::py::arg("newAngularAcceleration"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const math::Vector6d& _positions) -> math::Matrix6d {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::py::arg("positions"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const math::Vector6d& _q2,
              const math::Vector6d& _q1) -> math::Vector6d {
            return self->getPositionDifferencesStatic(_q2, _q1);
          },
          ::py::arg("q2"),
          ::py::arg("q1"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::FreeJoint::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "convertToPositions",
          +[](const Eigen::Isometry3d& _tf) -> math::Vector6d {
            return dart::dynamics::FreeJoint::convertToPositions(_tf);
          },
          ::py::arg("tf"))
      .def_static(
          "convertToTransform",
          +[](const math::Vector6d& _positions) -> Eigen::Isometry3d {
            return dart::dynamics::FreeJoint::convertToTransform(_positions);
          },
          ::py::arg("positions"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(joint, tf);
          },
          ::py::arg("joint"),
          ::py::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(joint, tf, withRespectTo);
          },
          ::py::arg("joint"),
          ::py::arg("tf"),
          ::py::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(bodyNode, tf);
          },
          ::py::arg("bodyNode"),
          ::py::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                bodyNode, tf, withRespectTo);
          },
          ::py::arg("bodyNode"),
          ::py::arg("tf"),
          ::py::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(skeleton, tf);
          },
          ::py::arg("skeleton"),
          ::py::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton, tf, withRespectTo);
          },
          ::py::arg("skeleton"),
          ::py::arg("tf"),
          ::py::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo,
              bool applyToAllRootBodies) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton, tf, withRespectTo, applyToAllRootBodies);
          },
          ::py::arg("skeleton"),
          ::py::arg("tf"),
          ::py::arg("withRespectTo"),
          ::py::arg("applyToAllRootBodies"));
}

} // namespace python
} // namespace dart
