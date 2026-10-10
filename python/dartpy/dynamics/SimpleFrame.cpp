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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/Entity.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/math/MathTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart {
namespace python {

void SimpleFrame(nb::module_& m)
{
  dartnb::dart_class<
      dart::dynamics::SimpleFrame,
      dart::dynamics::ShapeFrame,
      dart::dynamics::Detachable>(m, "SimpleFrame")
      .def(dartnb::init<>())
      .def(dartnb::init<dart::dynamics::Frame*>(), nb::arg("refFrame").none())
      .def(
          dartnb::init<dart::dynamics::Frame*, const std::string&>(),
          nb::arg("refFrame").none(),
          nb::arg("name"))
      .def(
          dartnb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          nb::arg("refFrame").none(),
          nb::arg("name"),
          nb::arg("relativeTransform"))
      .def(
          "setName",
          +[](dart::dynamics::SimpleFrame* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::SimpleFrame* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "clone",
          +[](const dart::dynamics::SimpleFrame* self)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->clone();
          })
      .def(
          "clone",
          +[](const dart::dynamics::SimpleFrame* self,
              dart::dynamics::Frame* _refFrame)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->clone(_refFrame);
          },
          nb::arg("refFrame").none())
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame) {
            self->copy(_otherFrame);
          },
          nb::arg("otherFrame").none())
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame) {
            self->copy(_otherFrame, _refFrame);
          },
          nb::arg("otherFrame").none(),
          nb::arg("refFrame").none())
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame,
              bool _copyProperties) {
            self->copy(_otherFrame, _refFrame, _copyProperties);
          },
          nb::arg("otherFrame").none(),
          nb::arg("refFrame").none(),
          nb::arg("copyProperties"))
      .def(
          "spawnChildSimpleFrame",
          +[](dart::dynamics::SimpleFrame* self)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->spawnChildSimpleFrame();
          })
      .def(
          "spawnChildSimpleFrame",
          +[](dart::dynamics::SimpleFrame* self, const std::string& name)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->spawnChildSimpleFrame(name);
          },
          nb::arg("name"))
      .def(
          "spawnChildSimpleFrame",
          +[](dart::dynamics::SimpleFrame* self,
              const std::string& name,
              const Eigen::Isometry3d& relativeTransform)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->spawnChildSimpleFrame(name, relativeTransform);
          },
          nb::arg("name"),
          nb::arg("relativeTransform"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newRelTransform) {
            self->setRelativeTransform(_newRelTransform);
          },
          nb::arg("newRelTransform"))
      .def(
          "setRelativeTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setRelativeTranslation(_newTranslation);
          },
          nb::arg("newTranslation"))
      .def(
          "setRelativeRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRelativeRotation(_newRotation);
          },
          nb::arg("newRotation"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform) {
            self->setTransform(_newTransform);
          },
          nb::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTransform(_newTransform, _withRespectTo);
          },
          nb::arg("newTransform"),
          nb::arg("withRespectTo").none())
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setTranslation(_newTranslation);
          },
          nb::arg("newTranslation"))
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTranslation(_newTranslation, _withRespectTo);
          },
          nb::arg("newTranslation"),
          nb::arg("withRespectTo").none())
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRotation(_newRotation);
          },
          nb::arg("newRotation"))
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setRotation(_newRotation, _withRespectTo);
          },
          nb::arg("newRotation"),
          nb::arg("withRespectTo").none())
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity) {
            self->setRelativeSpatialVelocity(_newSpatialVelocity);
          },
          nb::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                _newSpatialVelocity, _inCoordinatesOf);
          },
          nb::arg("newSpatialVelocity"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(_newSpatialAcceleration);
          },
          nb::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                _newSpatialAcceleration, _inCoordinatesOf);
          },
          nb::arg("newSpatialAcceleration"),
          nb::arg("inCoordinatesOf").none())
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self) {
            self->setClassicDerivatives();
          })
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity) {
            self->setClassicDerivatives(_linearVelocity);
          },
          nb::arg("linearVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity) {
            self->setClassicDerivatives(_linearVelocity, _angularVelocity);
          },
          nb::arg("linearVelocity"),
          nb::arg("angularVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity,
              const Eigen::Vector3d& _linearAcceleration) {
            self->setClassicDerivatives(
                _linearVelocity, _angularVelocity, _linearAcceleration);
          },
          nb::arg("linearVelocity"),
          nb::arg("angularVelocity"),
          nb::arg("linearAcceleration"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity,
              const Eigen::Vector3d& _linearAcceleration,
              const Eigen::Vector3d& _angularAcceleration) {
            self->setClassicDerivatives(
                _linearVelocity,
                _angularVelocity,
                _linearAcceleration,
                _angularAcceleration);
          },
          nb::arg("linearVelocity"),
          nb::arg("angularVelocity"),
          nb::arg("linearAcceleration"),
          nb::arg("angularAcceleration"));
}

} // namespace python
} // namespace dart
