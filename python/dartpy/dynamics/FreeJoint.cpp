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
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace dart {
namespace python {

void FreeJoint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::FreeJoint::Properties>(
      m, "FreeJointProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const dart::dynamics::GenericJoint<
              dart::math::SE3Space>::Properties&>(),
          ::pybind11::arg("properties"));

  ::pybind11::class_<
      dart::dynamics::FreeJoint,
      dart::dynamics::Joint
      //      dart::dynamics::GenericJoint<dart::math::SE3Space>
      >(m, "FreeJoint")
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
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::FreeJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::pybind11::arg("index"))
      .def(
          "setSpatialMotion",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d* newTransform,
              const dart::dynamics::Frame* withRespectTo,
              const Eigen::Vector6d* newSpatialVelocity,
              const dart::dynamics::Frame* velRelativeTo,
              const dart::dynamics::Frame* velInCoordinatesOf,
              const Eigen::Vector6d* newSpatialAcceleration,
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
          ::pybind11::arg("newTransform"),
          ::pybind11::arg("withRespectTo"),
          ::pybind11::arg("newSpatialVelocity"),
          ::pybind11::arg("velRelativeTo"),
          ::pybind11::arg("velInCoordinatesOf"),
          ::pybind11::arg("newSpatialAcceleration"),
          ::pybind11::arg("accRelativeTo"),
          ::pybind11::arg("accInCoordinatesOf"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setRelativeTransform(newTransform);
          },
          ::pybind11::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setTransform(newTransform);
          },
          ::pybind11::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform,
              const dart::dynamics::Frame* withRespectTo) {
            self->setTransform(newTransform, withRespectTo);
          },
          ::pybind11::arg("newTransform"),
          ::pybind11::arg("withRespectTo"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity) {
            self->setRelativeSpatialVelocity(newSpatialVelocity);
          },
          ::pybind11::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                newSpatialVelocity, inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialVelocity"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialVelocity(
                newSpatialVelocity, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialVelocity"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity) {
            self->setLinearVelocity(newLinearVelocity);
          },
          ::pybind11::arg("newLinearVelocity"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearVelocity(newLinearVelocity, relativeTo);
          },
          ::pybind11::arg("newLinearVelocity"),
          ::pybind11::arg("relativeTo"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearVelocity(
                newLinearVelocity, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newLinearVelocity"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity) {
            self->setAngularVelocity(newAngularVelocity);
          },
          ::pybind11::arg("newAngularVelocity"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularVelocity(newAngularVelocity, relativeTo);
          },
          ::pybind11::arg("newAngularVelocity"),
          ::pybind11::arg("relativeTo"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularVelocity(
                newAngularVelocity, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newAngularVelocity"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(newSpatialAcceleration);
          },
          ::pybind11::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                newSpatialAcceleration, inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialAcceleration"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialAcceleration(
                newSpatialAcceleration, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialAcceleration"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration) {
            self->setLinearAcceleration(newLinearAcceleration);
          },
          ::pybind11::arg("newLinearAcceleration"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearAcceleration(newLinearAcceleration, relativeTo);
          },
          ::pybind11::arg("newLinearAcceleration"),
          ::pybind11::arg("relativeTo"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearAcceleration(
                newLinearAcceleration, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newLinearAcceleration"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration) {
            self->setAngularAcceleration(newAngularAcceleration);
          },
          ::pybind11::arg("newAngularAcceleration"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularAcceleration(newAngularAcceleration, relativeTo);
          },
          ::pybind11::arg("newAngularAcceleration"),
          ::pybind11::arg("relativeTo"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularAcceleration(
                newAngularAcceleration, relativeTo, inCoordinatesOf);
          },
          ::pybind11::arg("newAngularAcceleration"),
          ::pybind11::arg("relativeTo"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& _positions) -> Eigen::Matrix6d {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& _q2,
              const Eigen::Vector6d& _q1) -> Eigen::Vector6d {
            return self->getPositionDifferencesStatic(_q2, _q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::FreeJoint::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def_static(
          "convertToPositions",
          +[](const Eigen::Isometry3d& _tf) -> Eigen::Vector6d {
            return dart::dynamics::FreeJoint::convertToPositions(_tf);
          },
          ::pybind11::arg("tf"))
      .def_static(
          "convertToTransform",
          +[](const Eigen::Vector6d& _positions) -> Eigen::Isometry3d {
            return dart::dynamics::FreeJoint::convertToTransform(_positions);
          },
          ::pybind11::arg("positions"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(joint, tf);
          },
          ::pybind11::arg("joint"),
          ::pybind11::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(joint, tf, withRespectTo);
          },
          ::pybind11::arg("joint"),
          ::pybind11::arg("tf"),
          ::pybind11::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(bodyNode, tf);
          },
          ::pybind11::arg("bodyNode"),
          ::pybind11::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                bodyNode, tf, withRespectTo);
          },
          ::pybind11::arg("bodyNode"),
          ::pybind11::arg("tf"),
          ::pybind11::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(skeleton, tf);
          },
          ::pybind11::arg("skeleton"),
          ::pybind11::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton, tf, withRespectTo);
          },
          ::pybind11::arg("skeleton"),
          ::pybind11::arg("tf"),
          ::pybind11::arg("withRespectTo"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo,
              bool applyToAllRootBodies) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton, tf, withRespectTo, applyToAllRootBodies);
          },
          ::pybind11::arg("skeleton"),
          ::pybind11::arg("tf"),
          ::pybind11::arg("withRespectTo"),
          ::pybind11::arg("applyToAllRootBodies"));
}

} // namespace python
} // namespace dart
