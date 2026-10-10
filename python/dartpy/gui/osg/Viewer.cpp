#include "detail/dart_nb.hpp"
#include "detail/eigen.hpp"

#include <nanobind/stl/tuple.h>

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

#include "gui/osg/drag_and_drop.hpp"
#include "gui/osg/ownership.hpp"

#include <dart/gui/osg/DragAndDrop.hpp>
#include <dart/gui/osg/InteractiveFrame.hpp>
#include <dart/gui/osg/OffscreenViewer.hpp>
#include <dart/gui/osg/Utils.hpp>
#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Entity.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/common/Subject.hpp>

#include <Eigen/Core>
#include <osg/BoundingSphere>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgGA/CameraManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

#include <memory>
#include <string>
#include <tuple>

#include <cstddef>

namespace dart {
namespace python {

void Viewer(nb::module_& m)
{
  dartnb::dart_class<osgViewer::View>(m, "osgViewer")
      .def(dartnb::gui::init<>())
      .def(
          "addEventHandler",
          +[](osgViewer::View* self, osgGA::GUIEventHandler* eventHandler) {
            dartnb::gui::retain(self, eventHandler, [self, eventHandler] {
              self->removeEventHandler(eventHandler);
            });
            self->addEventHandler(eventHandler);
          },
          nb::arg("eventHandler").none());

  auto viewer
      = dartnb::dart_class<
            dart::gui::osg::Viewer,
            osgViewer::View,
            dart::common::Subject>(m, "Viewer")
            .def(dartnb::gui::init<>())
            .def(
                dartnb::factory([](const Eigen::Vector4f& clearColor) {
                  return dartnb::gui::make<::dart::gui::osg::Viewer>(
                      gui::osg::eigToOsgVec4d(clearColor));
                }),
                nb::arg("clearColor"))
            .def(dartnb::gui::init<const osg::Vec4&>(), nb::arg("clearColor"))
            .def(
                "captureScreen",
                +[](dart::gui::osg::Viewer* self, const std::string& filename) {
                  self->captureScreen(filename);
                },
                nb::arg("filename"))
            .def(
                "setUpOffscreen",
                +[](dart::gui::osg::Viewer* self,
                    int width,
                    int height,
                    double fovYDeg,
                    double nearClip,
                    double farClip) -> bool {
                  dart::gui::osg::OffscreenSetup setup;
                  setup.width = width;
                  setup.height = height;
                  setup.fovYDeg = fovYDeg;
                  setup.nearClip = nearClip;
                  setup.farClip = farClip;
                  return dart::gui::osg::setUpOffscreenViewer(*self, setup);
                },
                nb::arg("width") = 640,
                nb::arg("height") = 480,
                nb::arg("fovYDeg") = 30.0,
                nb::arg("nearClip") = 0.1,
                nb::arg("farClip") = 1000.0)
            .def(
                "captureOffscreen",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& pngPath,
                    const Eigen::Vector3d& eye,
                    const Eigen::Vector3d& center,
                    const Eigen::Vector3d& up,
                    int width,
                    int height,
                    double fovYDeg,
                    double nearClip,
                    double farClip,
                    int warmupFrames) -> bool {
                  dart::gui::osg::OffscreenSetup setup;
                  setup.width = width;
                  setup.height = height;
                  setup.fovYDeg = fovYDeg;
                  setup.nearClip = nearClip;
                  setup.farClip = farClip;
                  return dart::gui::osg::captureOffscreen(
                      *self,
                      pngPath,
                      gui::osg::eigToOsgVec3f(eye),
                      gui::osg::eigToOsgVec3f(center),
                      gui::osg::eigToOsgVec3f(up),
                      setup,
                      warmupFrames);
                },
                nb::arg("pngPath"),
                nb::arg("eye"),
                nb::arg("center"),
                nb::arg("up"),
                nb::arg("width") = 640,
                nb::arg("height") = 480,
                nb::arg("fovYDeg") = 30.0,
                nb::arg("nearClip") = 0.1,
                nb::arg("farClip") = 1000.0,
                nb::arg("warmupFrames") = 10)
            .def(
                "record",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& directory) { self->record(directory); },
                nb::arg("directory"))
            .def(
                "record",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& directory,
                    const std::string& prefix) {
                  self->record(directory, prefix);
                },
                nb::arg("directory"),
                nb::arg("prefix"))
            .def(
                "record",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& directory,
                    const std::string& prefix,
                    bool restart) { self->record(directory, prefix, restart); },
                nb::arg("directory"),
                nb::arg("prefix"),
                nb::arg("restart"))
            .def(
                "record",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& directory,
                    const std::string& prefix,
                    bool restart,
                    std::size_t digits) {
                  self->record(directory, prefix, restart, digits);
                },
                nb::arg("directory"),
                nb::arg("prefix"),
                nb::arg("restart"),
                nb::arg("digits"))
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
                +[](dart::gui::osg::Viewer* self, bool on) {
                  self->switchDefaultEventHandler(on);
                },
                nb::arg("on"))
            .def(
                "switchHeadlights",
                +[](dart::gui::osg::Viewer* self, bool on) {
                  self->switchHeadlights(on);
                },
                nb::arg("on"))
            .def(
                "checkHeadlights",
                +[](const dart::gui::osg::Viewer* self) -> bool {
                  return self->checkHeadlights();
                })
            .def(
                "setLightingMode",
                &dart::gui::osg::Viewer::setLightingMode,
                nb::arg("lightingMode"))
            .def("getLightingMode", &dart::gui::osg::Viewer::getLightingMode)
            .def(
                "addWorldNode",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::WorldNode* newWorldNode) {
                  dartnb::gui::retain(self, newWorldNode, [self, newWorldNode] {
                    self->removeWorldNode(newWorldNode);
                  });
                  self->addWorldNode(newWorldNode);
                },
                nb::arg("newWorldNode").none())
            .def(
                "addWorldNode",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::WorldNode* newWorldNode,
                    bool active) {
                  dartnb::gui::retain(self, newWorldNode, [self, newWorldNode] {
                    self->removeWorldNode(newWorldNode);
                  });
                  self->addWorldNode(newWorldNode, active);
                },
                nb::arg("newWorldNode").none(),
                nb::arg("active"))
            .def(
                "removeWorldNode",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::WorldNode* oldWorldNode) {
                  self->removeWorldNode(oldWorldNode);
                  dartnb::gui::retire(self, oldWorldNode);
                },
                nb::arg("oldWorldNode").none())
            .def(
                "removeWorldNode",
                +[](dart::gui::osg::Viewer* self,
                    std::shared_ptr<dart::simulation::World> oldWorld) {
                  auto* node = self->getWorldNode(oldWorld);
                  self->removeWorldNode(oldWorld);
                  if (node)
                    dartnb::gui::retire(self, node);
                },
                nb::arg("oldWorld").none())
            .def(
                "addAttachment",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::ViewerAttachment* attachment) {
                  auto* previous
                      = attachment ? attachment->getViewer() : nullptr;
                  dartnb::gui::retain(self, attachment, [self, attachment] {
                    self->removeAttachment(attachment);
                  });
                  self->addAttachment(attachment);
                  if (previous && previous != self)
                    dartnb::gui::retire(previous, attachment);
                },
                nb::arg("attachment").none())
            .def(
                "removeAttachment",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::ViewerAttachment* attachment) {
                  self->removeAttachment(attachment);
                  dartnb::gui::retire(self, attachment);
                },
                nb::arg("attachment").none())
            .def(
                "setupDefaultLights",
                +[](dart::gui::osg::Viewer* self) {
                  self->setupDefaultLights();
                })
            .def(
                "setUpwardsDirection",
                +[](dart::gui::osg::Viewer* self, const osg::Vec3& up) {
                  self->setUpwardsDirection(up);
                },
                nb::arg("up"))
            .def(
                "setUpwardsDirection",
                +[](dart::gui::osg::Viewer* self, const Eigen::Vector3d& up) {
                  self->setUpwardsDirection(up);
                },
                nb::arg("up"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::WorldNode* node) {
                  self->setWorldNodeActive(node);
                },
                nb::arg("node").none())
            .def(
                "setWorldNodeActive",
                +[](dart::gui::osg::Viewer* self,
                    dart::gui::osg::WorldNode* node,
                    bool active) { self->setWorldNodeActive(node, active); },
                nb::arg("node").none(),
                nb::arg("active"))
            .def(
                "setWorldNodeActive",
                +[](dart::gui::osg::Viewer* self,
                    std::shared_ptr<dart::simulation::World> world) {
                  self->setWorldNodeActive(world);
                },
                nb::arg("world").none())
            .def(
                "setWorldNodeActive",
                +[](dart::gui::osg::Viewer* self,
                    std::shared_ptr<dart::simulation::World> world,
                    bool active) { self->setWorldNodeActive(world, active); },
                nb::arg("world").none(),
                nb::arg("active"))
            .def(
                "simulate",
                +[](dart::gui::osg::Viewer* self, bool on) {
                  self->simulate(on);
                },
                nb::arg("on"))
            .def(
                "isSimulating",
                +[](const dart::gui::osg::Viewer* self) -> bool {
                  return self->isSimulating();
                })
            .def(
                "allowSimulation",
                +[](dart::gui::osg::Viewer* self, bool allow) {
                  self->allowSimulation(allow);
                },
                nb::arg("allow"))
            .def(
                "isAllowingSimulation",
                +[](const dart::gui::osg::Viewer* self) -> bool {
                  return self->isAllowingSimulation();
                })
            .def(
                "enableDragAndDrop",
                [](nb::handle self, dart::gui::osg::InteractiveFrame* arg0) {
                  return dartnb::gui::watchDnd(
                      nb::cast<dart::gui::osg::Viewer*>(self)
                          ->enableDragAndDrop(arg0),
                      self);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"))
            .def(
                "enableDragAndDrop",
                [](nb::handle self, dart::dynamics::SimpleFrame* arg0) {
                  return dartnb::gui::watchDnd(
                      nb::cast<dart::gui::osg::Viewer*>(self)
                          ->enableDragAndDrop(arg0),
                      self);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"))
            .def(
                "enableDragAndDrop",
                [](nb::handle self,
                   dart::dynamics::SimpleFrame* arg0,
                   dart::dynamics::Shape* arg1) {
                  return dartnb::gui::watchDnd(
                      nb::cast<dart::gui::osg::Viewer*>(self)
                          ->enableDragAndDrop(arg0, arg1),
                      self);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"),
                nb::arg("shape"))
            .def(
                "enableDragAndDrop",
                [](nb::handle self,
                   dart::dynamics::BodyNode* arg0,
                   bool arg1,
                   bool arg2) {
                  return dartnb::gui::watchDnd(
                      nb::cast<dart::gui::osg::Viewer*>(self)
                          ->enableDragAndDrop(arg0, arg1, arg2),
                      self);
                },
                nb::rv_policy::reference_internal,
                nb::arg("bodyNode"),
                nb::arg("useExternalIK") = true,
                nb::arg("useWholeBody") = false)
            .def(
                "enableDragAndDrop",
                [](nb::handle self, dart::dynamics::Entity* arg0) {
                  return dartnb::gui::watchDnd(
                      nb::cast<dart::gui::osg::Viewer*>(self)
                          ->enableDragAndDrop(arg0),
                      self);
                },
                nb::rv_policy::reference_internal,
                nb::arg("entity"))
            .def(
                "disableDragAndDrop",
                nb::overload_cast<dart::gui::osg::InteractiveFrameDnD*>(
                    &dart::gui::osg::Viewer::disableDragAndDrop),
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                nb::overload_cast<dart::gui::osg::SimpleFrameDnD*>(
                    &dart::gui::osg::Viewer::disableDragAndDrop),
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                nb::overload_cast<dart::gui::osg::SimpleFrameShapeDnD*>(
                    &dart::gui::osg::Viewer::disableDragAndDrop),
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                nb::overload_cast<dart::gui::osg::BodyNodeDnD*>(
                    &dart::gui::osg::Viewer::disableDragAndDrop),
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                nb::overload_cast<dart::gui::osg::DragAndDrop*>(
                    &dart::gui::osg::Viewer::disableDragAndDrop),
                nb::arg("dnd"))
            .def(
                "getInstructions",
                +[](const dart::gui::osg::Viewer* self) -> const std::string& {
                  return self->getInstructions();
                },
                nb::rv_policy::reference_internal)
            .def(
                "addInstructionText",
                +[](dart::gui::osg::Viewer* self,
                    const std::string& _instruction) {
                  self->addInstructionText(_instruction);
                },
                nb::arg("instruction"))
            .def(
                "updateViewer",
                +[](dart::gui::osg::Viewer* self) { self->updateViewer(); })
            .def(
                "updateDragAndDrops",
                +[](dart::gui::osg::Viewer* self) {
                  self->updateDragAndDrops();
                })
            .def(
                "setVerticalFieldOfView",
                +[](dart::gui::osg::Viewer* self, double fov) {
                  self->setVerticalFieldOfView(fov);
                },
                nb::arg("fov"))
            .def(
                "getVerticalFieldOfView",
                +[](const dart::gui::osg::Viewer* self) -> double {
                  return self->getVerticalFieldOfView();
                })
            .def(
                "run",
                +[](dart::gui::osg::Viewer* self)
                    -> int { return self->run(); })
            .def(
                "frame", +[](dart::gui::osg::Viewer* self) { self->frame(); })
            .def(
                "frame",
                +[](dart::gui::osg::Viewer* self, double simulationTime) {
                  self->frame(simulationTime);
                })
            .def(
                "setUpViewInWindow",
                +[](dart::gui::osg::Viewer* self,
                    int x,
                    int y,
                    int width,
                    int height) {
                  self->setUpViewInWindow(x, y, width, height);
                })
            .def(
                "setCameraHomePosition",
                +[](dart::gui::osg::Viewer* self,
                    const Eigen::Vector3d& eye,
                    const Eigen::Vector3d& center,
                    const Eigen::Vector3d& up) {
                  self->getCameraManipulator()->setHomePosition(
                      gui::osg::eigToOsgVec3(eye),
                      gui::osg::eigToOsgVec3(center),
                      gui::osg::eigToOsgVec3(up));

                  self->setCameraManipulator(self->getCameraManipulator());
                })
            .def(
                "setCameraMode",
                &gui::osg::Viewer::setCameraMode,
                nb::arg("mode"))
            .def("getCameraMode", &gui::osg::Viewer::getCameraMode);

  // Agent-friendly default camera from a scene bounding sphere: a canonical 3/4
  // view framed to fill the vertical field of view, z-up. Returns
  // (eye, center, up) for use with Viewer.captureOffscreen.
  m.def(
      "defaultAgentCamera",
      +[](const Eigen::Vector3d& center,
          double radius,
          double fovYDeg,
          double azimuthDeg,
          double elevationDeg) {
        const ::osg::BoundingSphere bound(
            gui::osg::eigToOsgVec3f(center), static_cast<float>(radius));
        const dart::gui::osg::OffscreenCamera cam
            = dart::gui::osg::defaultAgentCamera(
                bound, fovYDeg, azimuthDeg, elevationDeg);
        return std::make_tuple(
            gui::osg::osgToEigVec3(cam.eye),
            gui::osg::osgToEigVec3(cam.center),
            gui::osg::osgToEigVec3(cam.up));
      },
      nb::arg("center"),
      nb::arg("radius"),
      nb::arg("fovYDeg") = 30.0,
      nb::arg("azimuthDeg") = 45.0,
      nb::arg("elevationDeg") = 30.0);

  nb::enum_<dart::gui::osg::Viewer::LightingMode>(
      viewer, "LightingMode", nb::is_arithmetic())
      .value("NO_LIGHT", dart::gui::osg::Viewer::NO_LIGHT)
      .value("HEADLIGHT", dart::gui::osg::Viewer::HEADLIGHT)
      .value("SKY_LIGHT", dart::gui::osg::Viewer::SKY_LIGHT);

  nb::enum_<dart::gui::osg::CameraMode>(m, "CameraMode", nb::is_arithmetic())
      .value("RGBA", dart::gui::osg::CameraMode::RGBA)
      .value("DEPTH", dart::gui::osg::CameraMode::DEPTH);

} // namespace python

} // namespace python
} // namespace dart
