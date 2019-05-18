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

void Viewer(pybind11::module& m)
{
  ::pybind11::class_<
      dart::gui::osg::Viewer,
      ::osg::ref_ptr<dart::gui::osg::Viewer>>(m, "Viewer")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const osg::Vec4&>(), ::pybind11::arg("clearColor"))
      .def(
          "captureScreen",
          +[](dart::gui::osg::Viewer* self, const std::string& filename) {
            self->captureScreen(filename);
          },
          ::pybind11::arg("filename"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self, const std::string& directory) {
            self->record(directory);
          },
          ::pybind11::arg("directory"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix) { self->record(directory, prefix); },
          ::pybind11::arg("directory"),
          ::pybind11::arg("prefix"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix,
              bool restart) { self->record(directory, prefix, restart); },
          ::pybind11::arg("directory"),
          ::pybind11::arg("prefix"),
          ::pybind11::arg("restart"))
      .def(
          "record",
          +[](dart::gui::osg::Viewer* self,
              const std::string& directory,
              const std::string& prefix,
              bool restart,
              std::size_t digits) {
            self->record(directory, prefix, restart, digits);
          },
          ::pybind11::arg("directory"),
          ::pybind11::arg("prefix"),
          ::pybind11::arg("restart"),
          ::pybind11::arg("digits"))
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
          ::pybind11::arg("on"))
      .def(
          "switchHeadlights",
          +[](dart::gui::osg::Viewer* self, bool _on) {
            self->switchHeadlights(_on);
          },
          ::pybind11::arg("on"))
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
          ::pybind11::arg("newWorldNode"))
      .def(
          "addWorldNode",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _newWorldNode,
              bool _active) { self->addWorldNode(_newWorldNode, _active); },
          ::pybind11::arg("newWorldNode"),
          ::pybind11::arg("active"))
      .def(
          "removeWorldNode",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _oldWorldNode) {
            self->removeWorldNode(_oldWorldNode);
          },
          ::pybind11::arg("oldWorldNode"))
      .def(
          "removeWorldNode",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _oldWorld) {
            self->removeWorldNode(_oldWorld);
          },
          ::pybind11::arg("oldWorld"))
      .def(
          "addAttachment",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::ViewerAttachment* _attachment) {
            self->addAttachment(_attachment);
          },
          ::pybind11::arg("attachment"))
      .def(
          "removeAttachment",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::ViewerAttachment* _attachment) {
            self->removeAttachment(_attachment);
          },
          ::pybind11::arg("attachment"))
      .def(
          "setupDefaultLights",
          +[](dart::gui::osg::Viewer* self) { self->setupDefaultLights(); })
      .def(
          "setUpwardsDirection",
          +[](dart::gui::osg::Viewer* self, const osg::Vec3& _up) {
            self->setUpwardsDirection(_up);
          },
          ::pybind11::arg("up"))
      .def(
          "setUpwardsDirection",
          +[](dart::gui::osg::Viewer* self, const Eigen::Vector3d& _up) {
            self->setUpwardsDirection(_up);
          },
          ::pybind11::arg("up"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::WorldNode* _node) {
            self->setWorldNodeActive(_node);
          },
          ::pybind11::arg("node"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::WorldNode* _node,
              bool _active) { self->setWorldNodeActive(_node, _active); },
          ::pybind11::arg("node"),
          ::pybind11::arg("active"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _world) {
            self->setWorldNodeActive(_world);
          },
          ::pybind11::arg("world"))
      .def(
          "setWorldNodeActive",
          +[](dart::gui::osg::Viewer* self,
              std::shared_ptr<dart::simulation::World> _world,
              bool _active) { self->setWorldNodeActive(_world, _active); },
          ::pybind11::arg("world"),
          ::pybind11::arg("active"))
      .def(
          "simulate",
          +[](dart::gui::osg::Viewer* self, bool _on) { self->simulate(_on); },
          ::pybind11::arg("on"))
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
          ::pybind11::arg("allow"))
      .def(
          "isAllowingSimulation",
          +[](const dart::gui::osg::Viewer* self) -> bool {
            return self->isAllowingSimulation();
          })
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::DragAndDrop* _dnd)
              -> bool { return self->disableDragAndDrop(_dnd); },
          ::pybind11::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::SimpleFrameDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::pybind11::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::SimpleFrameShapeDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::pybind11::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self,
              dart::gui::osg::InteractiveFrameDnD* _dnd) -> bool {
            return self->disableDragAndDrop(_dnd);
          },
          ::pybind11::arg("dnd"))
      .def(
          "disableDragAndDrop",
          +[](dart::gui::osg::Viewer* self, dart::gui::osg::BodyNodeDnD* _dnd)
              -> bool { return self->disableDragAndDrop(_dnd); },
          ::pybind11::arg("dnd"))
      .def(
          "getInstructions",
          +[](const dart::gui::osg::Viewer* self) -> const std::string& {
            return self->getInstructions();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "addInstructionText",
          +[](dart::gui::osg::Viewer* self, const std::string& _instruction) {
            self->addInstructionText(_instruction);
          },
          ::pybind11::arg("instruction"))
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
          ::pybind11::arg("fov"))
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
