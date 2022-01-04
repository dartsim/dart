/*
 * Copyright (c) 2011-2022, The DART development contributors
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

namespace py = pybind11;

namespace dart {
namespace python {

void SimpleFrame(py::module& m)
{
  ::py::class_<
      dart::dynamics::SimpleFrame,
      dart::dynamics::ShapeFrame,
      dart::dynamics::Detachable,
      std::shared_ptr<dart::dynamics::SimpleFrame>>(m, "SimpleFrame")
      .def(::py::init<>())
      .def(::py::init<dart::dynamics::Frame*>(), ::py::arg("refFrame"))
      .def(
          ::py::init<dart::dynamics::Frame*, const std::string&>(),
          ::py::arg("refFrame"),
          ::py::arg("name"))
      .def(
          ::py::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          ::py::arg("refFrame"),
          ::py::arg("name"),
          ::py::arg("relativeTransform"))
      .def(
          "setName",
          +[](dart::dynamics::SimpleFrame* self, const std::string& _name)
              -> const std::string& { return self->setName(_name); },
          ::py::return_value_policy::reference_internal,
          ::py::arg("name"))
      .def(
          "getName",
          +[](const dart::dynamics::SimpleFrame* self) -> const std::string& {
            return self->getName();
          },
          ::py::return_value_policy::reference_internal)
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
          ::py::arg("refFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame) {
            self->copy(_otherFrame);
          },
          ::py::arg("otherFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame) {
            self->copy(_otherFrame, _refFrame);
          },
          ::py::arg("otherFrame"),
          ::py::arg("refFrame"))
      .def(
          "copy",
          +[](dart::dynamics::SimpleFrame* self,
              const dart::dynamics::Frame* _otherFrame,
              dart::dynamics::Frame* _refFrame,
              bool _copyProperties) {
            self->copy(_otherFrame, _refFrame, _copyProperties);
          },
          ::py::arg("otherFrame"),
          ::py::arg("refFrame"),
          ::py::arg("copyProperties"))
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
          ::py::arg("name"))
      .def(
          "spawnChildSimpleFrame",
          +[](dart::dynamics::SimpleFrame* self,
              const std::string& name,
              const Eigen::Isometry3d& relativeTransform)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->spawnChildSimpleFrame(name, relativeTransform);
          },
          ::py::arg("name"),
          ::py::arg("relativeTransform"))
      .def(
          "setRelativeTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newRelTransform) {
            self->setRelativeTransform(_newRelTransform);
          },
          ::py::arg("newRelTransform"))
      .def(
          "setRelativeTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setRelativeTranslation(_newTranslation);
          },
          ::py::arg("newTranslation"))
      .def(
          "setRelativeRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRelativeRotation(_newRotation);
          },
          ::py::arg("newRotation"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform) {
            self->setTransform(_newTransform);
          },
          ::py::arg("newTransform"))
      .def(
          "setTransform",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Isometry3d& _newTransform,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTransform(_newTransform, _withRespectTo);
          },
          ::py::arg("newTransform"),
          ::py::arg("withRespectTo"))
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation) {
            self->setTranslation(_newTranslation);
          },
          ::py::arg("newTranslation"))
      .def(
          "setTranslation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _newTranslation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setTranslation(_newTranslation, _withRespectTo);
          },
          ::py::arg("newTranslation"),
          ::py::arg("withRespectTo"))
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation) {
            self->setRotation(_newRotation);
          },
          ::py::arg("newRotation"))
      .def(
          "setRotation",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Matrix3d& _newRotation,
              const dart::dynamics::Frame* _withRespectTo) {
            self->setRotation(_newRotation, _withRespectTo);
          },
          ::py::arg("newRotation"),
          ::py::arg("withRespectTo"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity) {
            self->setRelativeSpatialVelocity(_newSpatialVelocity);
          },
          ::py::arg("newSpatialVelocity"))
      .def(
          "setRelativeSpatialVelocity",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialVelocity,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialVelocity(
                _newSpatialVelocity, _inCoordinatesOf);
          },
          ::py::arg("newSpatialVelocity"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration) {
            self->setRelativeSpatialAcceleration(_newSpatialAcceleration);
          },
          ::py::arg("newSpatialAcceleration"))
      .def(
          "setRelativeSpatialAcceleration",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector6d& _newSpatialAcceleration,
              const dart::dynamics::Frame* _inCoordinatesOf) {
            self->setRelativeSpatialAcceleration(
                _newSpatialAcceleration, _inCoordinatesOf);
          },
          ::py::arg("newSpatialAcceleration"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("linearVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity) {
            self->setClassicDerivatives(_linearVelocity, _angularVelocity);
          },
          ::py::arg("linearVelocity"),
          ::py::arg("angularVelocity"))
      .def(
          "setClassicDerivatives",
          +[](dart::dynamics::SimpleFrame* self,
              const Eigen::Vector3d& _linearVelocity,
              const Eigen::Vector3d& _angularVelocity,
              const Eigen::Vector3d& _linearAcceleration) {
            self->setClassicDerivatives(
                _linearVelocity, _angularVelocity, _linearAcceleration);
          },
          ::py::arg("linearVelocity"),
          ::py::arg("angularVelocity"),
          ::py::arg("linearAcceleration"))
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
          ::py::arg("linearVelocity"),
          ::py::arg("angularVelocity"),
          ::py::arg("linearAcceleration"),
          ::py::arg("angularAcceleration"));
}

} // namespace python
} // namespace dart
