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
#include <dart/gui/osg/osg.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

void Viewer(py::module& m)
{
  ::py::class_<osgViewer::View, ::osg::ref_ptr<osgViewer::View>>(m, "osgViewer")
      .def(::py::init<>())
      .def(
          "addEventHandler",
          +[](osgViewer::View* self, osgGA::GUIEventHandler* eventHandler) {
            self->addEventHandler(eventHandler);
          },
          ::py::arg("eventHandler"));

  ::py::class_<
      dart::gui::osg::Viewer,
      osgViewer::View,
      dart::common::Subject,
      ::osg::ref_ptr<dart::gui::osg::Viewer>>(m, "Viewer")
      .def(::py::init<>())
      .def(
          ::py::init([](const Eigen::Vector4f& clearColor) {
            return new ::dart::gui::osg::Viewer(eigToOsgVec4f(clearColor));
          }),
          ::py::arg("clearColor"))
      .def(::py::init<const osg::Vec4&>(), ::py::arg("clearColor"))
      .def(
          "captureScreen",
          +[](dart::gui::osg::Viewer* self, const std::string& filename) {
            self->captureScreen(filename);
          },
          ::py::arg("filename"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self, const std::string& directory) {
            self->record(directory);
          },
          ::py::arg("directory"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix) { self->record(directory, prefix); },
          ::py::arg("directory"),
          ::py::arg("prefix"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix,
              bool restart) { self->record(directory, prefix, restart); },
          ::py::arg("directory"),
          ::py::arg("prefix"),
          ::py::arg("restart"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix,
              bool restart,
              std::size_t digits) {
            self->record(directory, prefix, restart, digits);
          },
          ::py::arg("directory"),
          ::py::arg("prefix"),
          ::py::arg("restart"),
          ::py::arg("digits"))
      .def(
          "pauseRecording",
          +[](dart::gui::osg::Viewer* self) { self->pauseRecording(); })
      .def(
          "isRecording",
          +[](const dart::gui::osg::Viewer* self) -> bool {
            return self->isRecording();
          })
      .def(
          "switchDefaultEventHandler",
          +[](dart::gui::osg::Viewer* self, bool _on) {
            self->switchDefaultEventHandler(_on);
          },
          ::py::arg("on"))
      .def(
          "switchHeadlights",
          +[](dart::gui::osg::Viewer* self, bool _on) {
            self->switchHeadlights(_on);
          },
          ::py::arg("on"))
      .def(
          "checkHeadlights",
          +[](const dart::gui::osg::Viewer* self) -> bool {
            return self->checkHeadlights();
          })
      .def(
          "addWorldNode",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _newWorldNode) {
            self->addWorldNode(_newWorldNode);
          },
          ::py::arg("newWorldNode"))
      .def(
          "addWorldNode",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _newWorldNode,
              bool _active) { self->addWorldNode(_newWorldNode, _active); },
          ::py::arg("newWorldNode"),
          ::py::arg("active"))
      .def(
          "removeWorldNode",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _oldWorldNode) {
            self->removeWorldNode(_oldWorldNode);
          },
          ::py::arg("oldWorldNode"))
      .def(
          "removeWorldNode",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _oldWorld) {
            self->removeWorldNode(_oldWorld);
          },
          ::py::arg("oldWorld"))
      .def(
          "addAttachment",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::ViewerAttachment* _attachment) {
            self->addAttachment(_attachment);
          },
          ::py::arg("attachment"))
      .def(
          "removeAttachment",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::ViewerAttachment* _attachment) {
            self->removeAttachment(_attachment);
          },
          ::py::arg("attachment"))
      .def(
          "setupDefaultLights",
          +[](dart::gui::osg::Viewer* self) { self->setupDefaultLights(); })
      .def(
          "setUpwardsDirection",
          +[](dart::gui::osg::Viewer* self, const osg::Vec3& _up) {
            self->setUpwardsDirection(_up);
          },
          ::py::arg("up"))
      .def(
          "setUpwardsDirection",
          +[](dart::gui::osg::Viewer* self, const Eigen::Vector3d& _up) {
            self->setUpwardsDirection(_up);
          },
          ::py::arg("up"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::WorldNode* _node) {
            self->setWorldNodeActive(_node);
          },
          ::py::arg("node"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _node,
              bool _active) { self->setWorldNodeActive(_node, _active); },
          ::py::arg("node"),
          ::py::arg("active"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _world) {
            self->setWorldNodeActive(_world);
          },
          ::py::arg("world"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _world,
              bool _active) { self->setWorldNodeActive(_world, _active); },
          ::py::arg("world"),
          ::py::arg("active"))
      .def(
          "simulate",
          +[](dart::gui::osg::Viewer* self, bool _on) { self->simulate(_on); },
          ::py::arg("on"))
      .def(
          "isSimulating",
          +[](const dart::gui::osg::Viewer* self) -> bool {
            return self->isSimulating();
          })
      .def(
          "allowSimulation",
          +[](dart::gui::osg::Viewer* self, bool _allow) {
            self->allowSimulation(_allow);
          },
          ::py::arg("allow"))
      .def(
          "isAllowingSimulation",
          +[](const dart::gui::osg::Viewer* self) -> bool {
            return self->isAllowingSimulation();
          })
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::dynamics::Entity* entity) -> dart::gui::osg::DragAndDrop* {
            return self->enableDragAndDrop(entity);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("entity"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::dynamics::SimpleFrame* frame)
              -> dart::gui::osg::SimpleFrameDnD* {
            return self->enableDragAndDrop(frame);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("frame"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::dynamics::SimpleFrame* frame,
              dart::dynamics::Shape* shape)
              -> dart::gui::osg::SimpleFrameShapeDnD* {
            return self->enableDragAndDrop(frame, shape);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("frame"),
          ::py::arg("shape"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::dynamics::BodyNode* bodyNode)
              -> dart::gui::osg::BodyNodeDnD* {
            return self->enableDragAndDrop(bodyNode);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("bodyNode"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::dynamics::BodyNode* bodyNode,
              bool useExternalIK) -> dart::gui::osg::BodyNodeDnD* {
            return self->enableDragAndDrop(bodyNode, useExternalIK);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("bodyNode"),
          ::py::arg("useExternalIK"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::dynamics::BodyNode* bodyNode,
              bool useExternalIK,
              bool useWholeBody) -> dart::gui::osg::BodyNodeDnD* {
            return self->enableDragAndDrop(
                bodyNode, useExternalIK, useWholeBody);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("bodyNode"),
          ::py::arg("useExternalIK"),
          ::py::arg("useWholeBody"))
      .def(
          "enableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::InteractiveFrame* frame)
              -> dart::gui::osg::InteractiveFrameDnD* {
            return self->enableDragAndDrop(frame);
          },
          py::return_value_policy::reference_internal,
          ::py::arg("frame"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::DragAndDrop* _dnd)
              -> bool { return self->disableDragAndDrop(_dnd); },
          ::py::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::SimpleFrameDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::py::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::SimpleFrameShapeDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::py::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::InteractiveFrameDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::py::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::BodyNodeDnD* _dnd)
              -> bool { return self->disableDragAndDrop(_dnd); },
          ::py::arg("dnd"))
      .def(
          "getInstructions",
          +[](const dart::gui::osg::Viewer* self) -> const std::string& {
            return self->getInstructions();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "addInstructionText",
          +[](dart::gui::osg::Viewer* self, const std::string& _instruction) {
            self->addInstructionText(_instruction);
          },
          ::py::arg("instruction"))
      .def(
          "updateViewer",
          +[](dart::gui::osg::Viewer* self) { self->updateViewer(); })
      .def(
          "updateDragAndDrops",
          +[](dart::gui::osg::Viewer* self) { self->updateDragAndDrops(); })
      .def(
          "setVerticalFieldOfView",
          +[](dart::gui::osg::Viewer* self, double fov) {
            self->setVerticalFieldOfView(fov);
          },
          ::py::arg("fov"))
      .def(
          "getVerticalFieldOfView",
          +[](const dart::gui::osg::Viewer* self) -> double {
            return self->getVerticalFieldOfView();
          })
      .def(
          "run",
          +[](dart::gui::osg::Viewer* self) -> int { return self->run(); })
      .def(
          "setUpViewInWindow",
          +[](dart::gui::osg::Viewer* self,
              int x,
              int y,
              int width,
              int height) { self->setUpViewInWindow(x, y, width, height); })
      .def(
          "setCameraHomePosition",
          +[](dart::gui::osg::Viewer* self,
              const Eigen::Vector3d& eye,
              const Eigen::Vector3d& center,
              const Eigen::Vector3d& up) {
            self->getCameraManipulator()->setHomePosition(
                eigToOsgVec3(eye), eigToOsgVec3(center), eigToOsgVec3(up));

            self->setCameraManipulator(self->getCameraManipulator());
          });
}

} // namespace python
} // namespace dart
