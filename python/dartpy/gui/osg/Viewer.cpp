/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/osg/All.hpp>

#include <dart/All.hpp>

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

  auto viewer
      = ::py::class_<
            dart::gui::Viewer,
            osgViewer::View,
            dart::common::Subject,
            ::osg::ref_ptr<dart::gui::Viewer>>(m, "Viewer")
            .def(::py::init<>())
            .def(
                ::py::init([](const Eigen::Vector4f& clearColor) {
                  return new ::dart::gui::Viewer(
                      gui::eigToOsgVec4d(clearColor));
                }),
                ::py::arg("clearColor"))
            .def(::py::init<const osg::Vec4&>(), ::py::arg("clearColor"))
            .def(
                "captureScreen",
                +[](dart::gui::Viewer* self, const std::string& filename) {
                  self->captureScreen(filename);
                },
                ::py::arg("filename"))
            .def(
                "record",
                +[](dart::gui::Viewer* self, const std::string& directory) {
                  self->record(directory);
                },
                ::py::arg("directory"))
            .def(
                "record",
                +[](dart::gui::Viewer* self,
                    const std::string& directory,
                    const std::string& prefix) {
                  self->record(directory, prefix);
                },
                ::py::arg("directory"),
                ::py::arg("prefix"))
            .def(
                "record",
                +[](dart::gui::Viewer* self,
                    const std::string& directory,
                    const std::string& prefix,
                    bool restart) { self->record(directory, prefix, restart); },
                ::py::arg("directory"),
                ::py::arg("prefix"),
                ::py::arg("restart"))
            .def(
                "record",
                +[](dart::gui::Viewer* self,
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
                +[](dart::gui::Viewer* self) { self->pauseRecording(); })
            .def(
                "isRecording",
                +[](const dart::gui::Viewer* self) -> bool {
                  return self->isRecording();
                })
            .def(
                "switchDefaultEventHandler",
                +[](dart::gui::Viewer* self, bool on) {
                  self->switchDefaultEventHandler(on);
                },
                ::py::arg("on"))
            .def(
                "switchHeadlights",
                +[](dart::gui::Viewer* self, bool on) {
                  self->switchHeadlights(on);
                },
                ::py::arg("on"))
            .def(
                "checkHeadlights",
                +[](const dart::gui::Viewer* self) -> bool {
                  return self->checkHeadlights();
                })
            .def(
                "setLightingMode",
                &dart::gui::Viewer::setLightingMode,
                ::py::arg("lightingMode"))
            .def("getLightingMode", &dart::gui::Viewer::getLightingMode)
            .def(
                "addWorldNode",
                +[](dart::gui::Viewer* self,
                    dart::gui::WorldNode* newWorldNode) {
                  self->addWorldNode(newWorldNode);
                },
                ::py::arg("newWorldNode"))
            .def(
                "addWorldNode",
                +[](dart::gui::Viewer* self,
                    dart::gui::WorldNode* newWorldNode,
                    bool active) { self->addWorldNode(newWorldNode, active); },
                ::py::arg("newWorldNode"),
                ::py::arg("active"))
            .def(
                "removeWorldNode",
                +[](dart::gui::Viewer* self,
                    dart::gui::WorldNode* oldWorldNode) {
                  self->removeWorldNode(oldWorldNode);
                },
                ::py::arg("oldWorldNode"))
            .def(
                "removeWorldNode",
                +[](dart::gui::Viewer* self,
                    std::shared_ptr<dart::simulation::World> oldWorld) {
                  self->removeWorldNode(oldWorld);
                },
                ::py::arg("oldWorld"))
            .def(
                "addAttachment",
                +[](dart::gui::Viewer* self,
                    dart::gui::ViewerAttachment* attachment) {
                  self->addAttachment(attachment);
                },
                ::py::arg("attachment"))
            .def(
                "removeAttachment",
                +[](dart::gui::Viewer* self,
                    dart::gui::ViewerAttachment* attachment) {
                  self->removeAttachment(attachment);
                },
                ::py::arg("attachment"))
            .def(
                "setupDefaultLights",
                +[](dart::gui::Viewer* self) { self->setupDefaultLights(); })
            .def(
                "setUpwardsDirection",
                +[](dart::gui::Viewer* self, const osg::Vec3& up) {
                  self->setUpwardsDirection(up);
                },
                ::py::arg("up"))
            .def(
                "setUpwardsDirection",
                +[](dart::gui::Viewer* self, const Eigen::Vector3d& up) {
                  self->setUpwardsDirection(up);
                },
                ::py::arg("up"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::Viewer* self, dart::gui::WorldNode* node) {
                  self->setWorldNodeActive(node);
                },
                ::py::arg("node"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::Viewer* self,
                    dart::gui::WorldNode* node,
                    bool active) { self->setWorldNodeActive(node, active); },
                ::py::arg("node"),
                ::py::arg("active"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::Viewer* self,
                    std::shared_ptr<dart::simulation::World> world) {
                  self->setWorldNodeActive(world);
                },
                ::py::arg("world"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::Viewer* self,
                    std::shared_ptr<dart::simulation::World> world,
                    bool active) { self->setWorldNodeActive(world, active); },
                ::py::arg("world"),
                ::py::arg("active"))
            .def(
                "simulate",
                +[](dart::gui::Viewer* self, bool on) { self->simulate(on); },
                ::py::arg("on"))
            .def(
                "isSimulating",
                +[](const dart::gui::Viewer* self) -> bool {
                  return self->isSimulating();
                })
            .def(
                "allowSimulation",
                +[](dart::gui::Viewer* self, bool allow) {
                  self->allowSimulation(allow);
                },
                ::py::arg("allow"))
            .def(
                "isAllowingSimulation",
                +[](const dart::gui::Viewer* self) -> bool {
                  return self->isAllowingSimulation();
                })
            .def(
                "enableDragAndDrop",
                ::py::overload_cast<dart::gui::InteractiveFrame*>(
                    &dart::gui::Viewer::enableDragAndDrop),
                ::py::return_value_policy::reference_internal,
                ::py::arg("frame"))
            .def(
                "enableDragAndDrop",
                ::py::overload_cast<dart::dynamics::SimpleFrame*>(
                    &dart::gui::Viewer::enableDragAndDrop),
                ::py::return_value_policy::reference_internal,
                ::py::arg("frame"))
            .def(
                "enableDragAndDrop",
                ::py::overload_cast<
                    dart::dynamics::SimpleFrame*,
                    dart::dynamics::Shape*>(
                    &dart::gui::Viewer::enableDragAndDrop),
                ::py::return_value_policy::reference_internal,
                ::py::arg("frame"),
                ::py::arg("shape"))
            .def(
                "enableDragAndDrop",
                ::py::overload_cast<dart::dynamics::BodyNode*, bool, bool>(
                    &dart::gui::Viewer::enableDragAndDrop),
                ::py::return_value_policy::reference_internal,
                ::py::arg("bodyNode"),
                ::py::arg_v("useExternalIK", true),
                ::py::arg_v("useWholeBody", false))
            .def(
                "enableDragAndDrop",
                ::py::overload_cast<dart::dynamics::Entity*>(
                    &dart::gui::Viewer::enableDragAndDrop),
                ::py::return_value_policy::reference_internal,
                ::py::arg("entity"))
            .def(
                "disableDragAndDrop",
                ::py::overload_cast<dart::gui::InteractiveFrameDnD*>(
                    &dart::gui::Viewer::disableDragAndDrop),
                ::py::arg("dnd"))
            .def(
                "disableDragAndDrop",
                ::py::overload_cast<dart::gui::SimpleFrameDnD*>(
                    &dart::gui::Viewer::disableDragAndDrop),
                ::py::arg("dnd"))
            .def(
                "disableDragAndDrop",
                ::py::overload_cast<dart::gui::SimpleFrameShapeDnD*>(
                    &dart::gui::Viewer::disableDragAndDrop),
                ::py::arg("dnd"))
            .def(
                "disableDragAndDrop",
                ::py::overload_cast<dart::gui::BodyNodeDnD*>(
                    &dart::gui::Viewer::disableDragAndDrop),
                ::py::arg("dnd"))
            .def(
                "disableDragAndDrop",
                ::py::overload_cast<dart::gui::DragAndDrop*>(
                    &dart::gui::Viewer::disableDragAndDrop),
                ::py::arg("dnd"))
            .def(
                "getInstructions",
                +[](const dart::gui::Viewer* self) -> const std::string& {
                  return self->getInstructions();
                },
                ::py::return_value_policy::reference_internal)
            .def(
                "addInstructionText",
                +[](dart::gui::Viewer* self, const std::string& _instruction) {
                  self->addInstructionText(_instruction);
                },
                ::py::arg("instruction"))
            .def(
                "updateViewer",
                +[](dart::gui::Viewer* self) { self->updateViewer(); })
            .def(
                "updateDragAndDrops",
                +[](dart::gui::Viewer* self) { self->updateDragAndDrops(); })
            .def(
                "setVerticalFieldOfView",
                +[](dart::gui::Viewer* self, double fov) {
                  self->setVerticalFieldOfView(fov);
                },
                ::py::arg("fov"))
            .def(
                "getVerticalFieldOfView",
                +[](const dart::gui::Viewer* self) -> double {
                  return self->getVerticalFieldOfView();
                })
            .def(
                "run",
                +[](dart::gui::Viewer* self) -> int { return self->run(); })
            .def(
                "frame", +[](dart::gui::Viewer* self) { self->frame(); })
            .def(
                "frame",
                +[](dart::gui::Viewer* self,
                    double simulationTime) { self->frame(simulationTime); })
            .def(
                "setUpViewInWindow",
                +[](dart::gui::Viewer* self,
                    int x,
                    int y,
                    int width,
                    int height) {
                  self->setUpViewInWindow(x, y, width, height);
                })
            .def(
                "setCameraHomePosition",
                +[](dart::gui::Viewer* self,
                    const Eigen::Vector3d& eye,
                    const Eigen::Vector3d& center,
                    const Eigen::Vector3d& up) {
                  self->getCameraManipulator()->setHomePosition(
                      gui::eigToOsgVec3(eye),
                      gui::eigToOsgVec3(center),
                      gui::eigToOsgVec3(up));

                  self->setCameraManipulator(self->getCameraManipulator());
                })
            .def("setCameraMode", &gui::Viewer::setCameraMode, py::arg("mode"))
            .def("getCameraMode", &gui::Viewer::getCameraMode);

  ::py::enum_<dart::gui::Viewer::LightingMode>(viewer, "LightingMode")
      .value("NO_LIGHT", dart::gui::Viewer::NO_LIGHT)
      .value("HEADLIGHT", dart::gui::Viewer::HEADLIGHT)
      .value("SKY_LIGHT", dart::gui::Viewer::SKY_LIGHT);

  ::py::enum_<dart::gui::CameraMode>(m, "CameraMode")
      .value("RGBA", dart::gui::CameraMode::RGBA)
      .value("DEPTH", dart::gui::CameraMode::DEPTH);

}  // namespace python

}  // namespace python
}  // namespace dart
