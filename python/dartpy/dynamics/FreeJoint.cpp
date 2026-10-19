// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/frame.hpp"

#include <nanobind/stl/optional.h>

#include <optional>

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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/GenericJoint.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/ConfigurationSpace.hpp>
#include <dart/math/MathTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

void FreeJoint(nb::module_& m)
{
  dartnb::dart_class<
      dart::dynamics::FreeJoint::Properties,
      dart::dynamics::GenericJoint<math::SE3Space>::Properties>(
      m, "FreeJointProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<const dart::dynamics::GenericJoint<
              dart::math::SE3Space>::Properties&>(),
          nb::arg("properties"));

  dartnb::dart_class<
      dart::dynamics::FreeJoint,
      dart::dynamics::GenericJoint<dart::math::SE3Space>>(m, "FreeJoint")
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
          nb::rv_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::FreeJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          nb::arg("index"))
      .def(
          "setSpatialMotion",
          +[](dart::dynamics::FreeJoint* self,
              const std::optional<Eigen::Isometry3d>& newTransform,
              const dart::dynamics::Frame* withRespectTo,
              const std::optional<Eigen::Vector6d>& newSpatialVelocity,
              const dart::dynamics::Frame* velRelativeTo,
              const dart::dynamics::Frame* velInCoordinatesOf,
              const std::optional<Eigen::Vector6d>& newSpatialAcceleration,
              const dart::dynamics::Frame* accRelativeTo,
              const dart::dynamics::Frame* accInCoordinatesOf) {
            // C++ ignores the frames of a motion that is not set, so None is
            // accepted for them; a motion that is set needs its frames.
            const auto require = [](bool motionSet,
                                    const dart::dynamics::Frame* frame,
                                    const char* message) {
              if (motionSet && !frame)
                throw nb::type_error(message);
            };
            require(
                newTransform.has_value(),
                withRespectTo,
                "withRespectTo is required when newTransform is set");
            require(
                newSpatialVelocity.has_value(),
                velRelativeTo,
                "velRelativeTo is required when newSpatialVelocity is set");
            require(
                newSpatialVelocity.has_value(),
                velInCoordinatesOf,
                "velInCoordinatesOf is required when newSpatialVelocity is "
                "set");
            require(
                newSpatialAcceleration.has_value(),
                accRelativeTo,
                "accRelativeTo is required when newSpatialAcceleration is set");
            require(
                newSpatialAcceleration.has_value(),
                accInCoordinatesOf,
                "accInCoordinatesOf is required when newSpatialAcceleration "
                "is set");
            self->setSpatialMotion(
                newTransform ? &*newTransform : nullptr,
                withRespectTo,
                newSpatialVelocity ? &*newSpatialVelocity : nullptr,
                velRelativeTo,
                velInCoordinatesOf,
                newSpatialAcceleration ? &*newSpatialAcceleration : nullptr,
                accRelativeTo,
                accInCoordinatesOf);
          },
          nb::arg("newTransform").none(),
          nb::arg("withRespectTo").none(),
          nb::arg("newSpatialVelocity").none(),
          nb::arg("velRelativeTo").none(),
          nb::arg("velInCoordinatesOf").none(),
          nb::arg("newSpatialAcceleration").none(),
          nb::arg("accRelativeTo").none(),
          nb::arg("accInCoordinatesOf").none())
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setRelativeTransform(newTransform);
          },
          nb::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform) {
            self->setTransform(newTransform);
          },
          nb::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Isometry3d& newTransform,
              const dart::dynamics::Frame* withRespectTo) {
            self->setTransform(
                newTransform, dartnb::frameOrWorld(withRespectTo));
          },
          nb::arg("newTransform"),
          nb::arg("withRespectTo").none())
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity) {
            self->setRelativeSpatialVelocity(newSpatialVelocity);
          },
          nb::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                newSpatialVelocity, inCoordinatesOf);
          },
          nb::arg("newSpatialVelocity"),
          nb::arg("inCoordinatesOf"))
      .def(
          "setSpatialVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialVelocity(
                newSpatialVelocity, relativeTo, inCoordinatesOf);
          },
          nb::arg("newSpatialVelocity"),
          nb::arg("relativeTo"),
          nb::arg("inCoordinatesOf"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity) {
            self->setLinearVelocity(newLinearVelocity);
          },
          nb::arg("newLinearVelocity"))
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearVelocity(
                newLinearVelocity, dartnb::frameOrWorld(relativeTo));
          },
          nb::arg("newLinearVelocity"),
          nb::arg("relativeTo").none())
      .def(
          "setLinearVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearVelocity(
                newLinearVelocity,
                dartnb::frameOrWorld(relativeTo),
                dartnb::frameOrWorld(inCoordinatesOf));
          },
          nb::arg("newLinearVelocity"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity) {
            self->setAngularVelocity(newAngularVelocity);
          },
          nb::arg("newAngularVelocity"))
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularVelocity(
                newAngularVelocity, dartnb::frameOrWorld(relativeTo));
          },
          nb::arg("newAngularVelocity"),
          nb::arg("relativeTo").none())
      .def(
          "setAngularVelocity",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularVelocity,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularVelocity(
                newAngularVelocity,
                dartnb::frameOrWorld(relativeTo),
                dartnb::frameOrWorld(inCoordinatesOf));
          },
          nb::arg("newAngularVelocity"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(newSpatialAcceleration);
          },
          nb::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                newSpatialAcceleration, inCoordinatesOf);
          },
          nb::arg("newSpatialAcceleration"),
          nb::arg("inCoordinatesOf"))
      .def(
          "setSpatialAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& newSpatialAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setSpatialAcceleration(
                newSpatialAcceleration, relativeTo, inCoordinatesOf);
          },
          nb::arg("newSpatialAcceleration"),
          nb::arg("relativeTo"),
          nb::arg("inCoordinatesOf"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration) {
            self->setLinearAcceleration(newLinearAcceleration);
          },
          nb::arg("newLinearAcceleration"))
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setLinearAcceleration(
                newLinearAcceleration, dartnb::frameOrWorld(relativeTo));
          },
          nb::arg("newLinearAcceleration"),
          nb::arg("relativeTo").none())
      .def(
          "setLinearAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newLinearAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setLinearAcceleration(
                newLinearAcceleration,
                dartnb::frameOrWorld(relativeTo),
                dartnb::frameOrWorld(inCoordinatesOf));
          },
          nb::arg("newLinearAcceleration"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration) {
            self->setAngularAcceleration(newAngularAcceleration);
          },
          nb::arg("newAngularAcceleration"))
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo) {
            self->setAngularAcceleration(
                newAngularAcceleration, dartnb::frameOrWorld(relativeTo));
          },
          nb::arg("newAngularAcceleration"),
          nb::arg("relativeTo").none())
      .def(
          "setAngularAcceleration",
          +[](dart::dynamics::FreeJoint* self,
              const Eigen::Vector3d& newAngularAcceleration,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) {
            self->setAngularAcceleration(
                newAngularAcceleration,
                dartnb::frameOrWorld(relativeTo),
                dartnb::frameOrWorld(inCoordinatesOf));
          },
          nb::arg("newAngularAcceleration"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& _positions) -> Eigen::Matrix6d {
            return self->getRelativeJacobianStatic(_positions);
          },
          nb::arg("positions"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::FreeJoint* self,
              const Eigen::Vector6d& _q2,
              const Eigen::Vector6d& _q1) -> Eigen::Vector6d {
            return self->getPositionDifferencesStatic(_q2, _q1);
          },
          nb::arg("q2"),
          nb::arg("q1"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::FreeJoint::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "convertToPositions",
          +[](const Eigen::Isometry3d& _tf) -> Eigen::Vector6d {
            return dart::dynamics::FreeJoint::convertToPositions(_tf);
          },
          nb::arg("tf"))
      .def_static(
          "convertToTransform",
          +[](const Eigen::Vector6d& _positions) -> Eigen::Isometry3d {
            return dart::dynamics::FreeJoint::convertToTransform(_positions);
          },
          nb::arg("positions"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(joint, tf);
          },
          nb::arg("joint").none(),
          nb::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Joint* joint,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                joint, tf, dartnb::frameOrWorld(withRespectTo));
          },
          nb::arg("joint").none(),
          nb::arg("tf"),
          nb::arg("withRespectTo").none())
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(bodyNode, tf);
          },
          nb::arg("bodyNode").none(),
          nb::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::BodyNode* bodyNode,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                bodyNode, tf, dartnb::frameOrWorld(withRespectTo));
          },
          nb::arg("bodyNode").none(),
          nb::arg("tf"),
          nb::arg("withRespectTo").none())
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton, const Eigen::Isometry3d& tf) {
            dart::dynamics::FreeJoint::setTransformOf(skeleton, tf);
          },
          nb::arg("skeleton").none(),
          nb::arg("tf"))
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton, tf, dartnb::frameOrWorld(withRespectTo));
          },
          nb::arg("skeleton").none(),
          nb::arg("tf"),
          nb::arg("withRespectTo").none())
      .def_static(
          "setTransformOf",
          +[](dart::dynamics::Skeleton* skeleton,
              const Eigen::Isometry3d& tf,
              const dart::dynamics::Frame* withRespectTo,
              bool applyToAllRootBodies) {
            dart::dynamics::FreeJoint::setTransformOf(
                skeleton,
                tf,
                dartnb::frameOrWorld(withRespectTo),
                applyToAllRootBodies);
          },
          nb::arg("skeleton").none(),
          nb::arg("tf"),
          nb::arg("withRespectTo").none(),
          nb::arg("applyToAllRootBodies"));
}

} // namespace python
} // namespace dart
