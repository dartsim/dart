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

void SimpleFrame(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::SimpleFrame,
      dart::dynamics::ShapeFrame,
      dart::dynamics::Detachable,
      std::shared_ptr<dart::dynamics::SimpleFrame> >(m, "SimpleFrame")
      .def(
          ::pybind11::init<dart::dynamics::Frame*>(),
          ::pybind11::arg("refFrame"))
      .def(
          ::pybind11::init<dart::dynamics::Frame*, const std::string&>(),
          ::pybind11::arg("refFrame"),
          ::pybind11::arg("name"))
      .def(
          ::pybind11::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          ::pybind11::arg("refFrame"),
          ::pybind11::arg("name"),
          ::pybind11::arg("relativeTransform"))
      .def(
          "setName",
          +[](dart::dynamics::SimpleFrame* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::SimpleFrame* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
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
          ::pybind11::arg("refFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame) {
            self->copy(_otherFrame);
          },
          ::pybind11::arg("otherFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame) {
            self->copy(_otherFrame, _refFrame);
          },
          ::pybind11::arg("otherFrame"),
          ::pybind11::arg("refFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame,
              bool _copyProperties) {
            self->copy(_otherFrame, _refFrame, _copyProperties);
          },
          ::pybind11::arg("otherFrame"),
          ::pybind11::arg("refFrame"),
          ::pybind11::arg("copyProperties"))
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
          ::pybind11::arg("name"))
      .def(
          "spawnChildSimpleFrame",
          +[](dart::dynamics::SimpleFrame* self,
              const std::string& name,
              const Eigen::Isometry3d& relativeTransform)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->spawnChildSimpleFrame(name, relativeTransform);
          },
          ::pybind11::arg("name"),
          ::pybind11::arg("relativeTransform"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newRelTransform) {
            self->setRelativeTransform(_newRelTransform);
          },
          ::pybind11::arg("newRelTransform"))
      .def(
          "setRelativeTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setRelativeTranslation(_newTranslation);
          },
          ::pybind11::arg("newTranslation"))
      .def(
          "setRelativeRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRelativeRotation(_newRotation);
          },
          ::pybind11::arg("newRotation"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform) {
            self->setTransform(_newTransform);
          },
          ::pybind11::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTransform(_newTransform, _withRespectTo);
          },
          ::pybind11::arg("newTransform"),
          ::pybind11::arg("withRespectTo"))
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setTranslation(_newTranslation);
          },
          ::pybind11::arg("newTranslation"))
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTranslation(_newTranslation, _withRespectTo);
          },
          ::pybind11::arg("newTranslation"),
          ::pybind11::arg("withRespectTo"))
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRotation(_newRotation);
          },
          ::pybind11::arg("newRotation"))
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setRotation(_newRotation, _withRespectTo);
          },
          ::pybind11::arg("newRotation"),
          ::pybind11::arg("withRespectTo"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity) {
            self->setRelativeSpatialVelocity(_newSpatialVelocity);
          },
          ::pybind11::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                _newSpatialVelocity, _inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialVelocity"),
          ::pybind11::arg("inCoordinatesOf"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(_newSpatialAcceleration);
          },
          ::pybind11::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                _newSpatialAcceleration, _inCoordinatesOf);
          },
          ::pybind11::arg("newSpatialAcceleration"),
          ::pybind11::arg("inCoordinatesOf"))
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
          ::pybind11::arg("linearVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity) {
            self->setClassicDerivatives(_linearVelocity, _angularVelocity);
          },
          ::pybind11::arg("linearVelocity"),
          ::pybind11::arg("angularVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity,
              const Eigen::Vector3d& _linearAcceleration) {
            self->setClassicDerivatives(
                _linearVelocity, _angularVelocity, _linearAcceleration);
          },
          ::pybind11::arg("linearVelocity"),
          ::pybind11::arg("angularVelocity"),
          ::pybind11::arg("linearAcceleration"))
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
          ::pybind11::arg("linearVelocity"),
          ::pybind11::arg("angularVelocity"),
          ::pybind11::arg("linearAcceleration"),
          ::pybind11::arg("angularAcceleration"));
}

} // namespace python
} // namespace dart
