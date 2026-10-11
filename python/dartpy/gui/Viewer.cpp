// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"
#include "gui/ownership.hpp"

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

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/DefaultEventHandler.hpp>
#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/GridVisual.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/InteractiveFrame.hpp>
#include <dart/gui/RealTimeWorldNode.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

namespace dart {
namespace python {
nb::tuple dart_gui_interactions(nb::module_& m);
nb::object dart_gui_owned_drag(
    const std::shared_ptr<gui::DragAndDrop>& drag, const nb::tuple& ownership);
namespace {

template <class Base>
class PyWorldNode : public Base
{
public:
  NB_TRAMPOLINE(Base);
  void refresh() override
  {
    NB_OVERRIDE(refresh);
  }
  void customPreRefresh() override
  {
    NB_OVERRIDE(customPreRefresh);
  }
  void customPostRefresh() override
  {
    NB_OVERRIDE(customPostRefresh);
  }
  void customPreStep() override
  {
    NB_OVERRIDE(customPreStep);
  }
  void customPostStep() override
  {
    NB_OVERRIDE(customPostStep);
  }
  void setupViewer() override
  {
    NB_OVERRIDE(setupViewer);
  }
};

class PyViewer : public gui::Viewer
{
public:
  NB_TRAMPOLINE(gui::Viewer);
  void updateViewer() override
  {
    NB_OVERRIDE(updateViewer);
  }
};

class PyViewerAttachment : public gui::ViewerAttachment
{
public:
  NB_TRAMPOLINE(gui::ViewerAttachment);
  void refresh() override
  {
    NB_OVERRIDE_PURE(refresh);
  }
  void customAttach(gui::Viewer* viewer) override
  {
    NB_OVERRIDE(customAttach, viewer);
  }
};

class ViewerAttachmentAccess : public gui::ViewerAttachment
{
public:
  using gui::ViewerAttachment::customAttach;
};

class PyImGuiWidget : public gui::ImGuiWidget
{
public:
  NB_TRAMPOLINE(gui::ImGuiWidget);
  void render() override
  {
    NB_OVERRIDE_PURE(render);
  }
};

} // namespace

void dart_gui_filament(nb::module_& m)
{
  const auto dragOwners = dart_gui_interactions(m);
  nb::enum_<gui::CameraMode>(m, "CameraMode")
      .value("RGBA", gui::CameraMode::RGBA)
      .value("DEPTH", gui::CameraMode::DEPTH);

  dartnb::dart_class<gui::WorldNode, PyWorldNode<gui::WorldNode>>(
      m, "WorldNode")
      .def(
          dartnb::init<std::shared_ptr<simulation::World>>(),
          nb::arg("world").none() = nullptr)
      .def("setWorld", &gui::WorldNode::setWorld, nb::arg("world").none())
      .def("getWorld", &gui::WorldNode::getWorld)
      .def("refresh", &gui::WorldNode::refresh)
      .def("customPreRefresh", &gui::WorldNode::customPreRefresh)
      .def("customPostRefresh", &gui::WorldNode::customPostRefresh)
      .def("customPreStep", &gui::WorldNode::customPreStep)
      .def("customPostStep", &gui::WorldNode::customPostStep)
      .def("setupViewer", &gui::WorldNode::setupViewer)
      .def("getViewer", &gui::WorldNode::getViewer, nb::rv_policy::reference)
      .def("simulate", &gui::WorldNode::simulate, nb::arg("on"))
      .def("isSimulating", &gui::WorldNode::isSimulating)
      .def(
          "setNumStepsPerCycle",
          &gui::WorldNode::setNumStepsPerCycle,
          nb::arg("steps"))
      .def("getNumStepsPerCycle", &gui::WorldNode::getNumStepsPerCycle)
      .def(
          "setShadowsEnabled",
          &gui::WorldNode::setShadowsEnabled,
          nb::arg("enabled"))
      .def("isShadowed", &gui::WorldNode::isShadowed);

  dartnb::dart_class<
      gui::RealTimeWorldNode,
      gui::WorldNode,
      PyWorldNode<gui::RealTimeWorldNode>>(m, "RealTimeWorldNode")
      .def(
          dartnb::
              init<const std::shared_ptr<simulation::World>&, double, double>(),
          nb::arg("world").none() = nullptr,
          nb::arg("targetFrequency") = 60.0,
          nb::arg("targetRealTimeFactor") = 1.0)
      .def(
          "setTargetFrequency",
          &gui::RealTimeWorldNode::setTargetFrequency,
          nb::arg("targetFrequency"))
      .def("getTargetFrequency", &gui::RealTimeWorldNode::getTargetFrequency)
      .def(
          "setTargetRealTimeFactor",
          &gui::RealTimeWorldNode::setTargetRealTimeFactor,
          nb::arg("targetRealTimeFactor"))
      .def(
          "getTargetRealTimeFactor",
          &gui::RealTimeWorldNode::getTargetRealTimeFactor)
      .def(
          "getLastRealTimeFactor",
          &gui::RealTimeWorldNode::getLastRealTimeFactor)
      .def(
          "getSmoothedRealTimeFactor",
          &gui::RealTimeWorldNode::getSmoothedRealTimeFactor)
      .def(
          "getLowestRealTimeFactor",
          &gui::RealTimeWorldNode::getLowestRealTimeFactor)
      .def(
          "getHighestRealTimeFactor",
          &gui::RealTimeWorldNode::getHighestRealTimeFactor);

  dartnb::dart_class<gui::ViewerAttachment, PyViewerAttachment>(
      m, "ViewerAttachment")
      .def(dartnb::init<>())
      .def("refresh", &gui::ViewerAttachment::refresh)
      .def(
          "customAttach",
          &ViewerAttachmentAccess::customAttach,
          nb::arg("viewer").none())
      .def(
          "getViewer",
          &gui::ViewerAttachment::getViewer,
          nb::rv_policy::reference);

  dartnb::dart_class<gui::ImGuiWidget, PyImGuiWidget>(m, "ImGuiWidget")
      .def(dartnb::init<>())
      .def("render", &gui::ImGuiWidget::render)
      .def("setVisible", &gui::ImGuiWidget::setVisible, nb::arg("visible"))
      .def("isVisible", &gui::ImGuiWidget::isVisible)
      .def("toggleVisible", &gui::ImGuiWidget::toggleVisible)
      .def("show", &gui::ImGuiWidget::show)
      .def("hide", &gui::ImGuiWidget::hide);

  auto grid
      = dartnb::dart_class<gui::GridVisual, gui::ViewerAttachment>(
            m, "GridVisual")
            .def(dartnb::init<>())
            .def("setNumCells", &gui::GridVisual::setNumCells)
            .def("getNumCells", &gui::GridVisual::getNumCells)
            .def("setMinorLineStepSize", &gui::GridVisual::setMinorLineStepSize)
            .def("getMinorLineStepSize", &gui::GridVisual::getMinorLineStepSize)
            .def(
                "setNumMinorLinesPerMajorLine",
                &gui::GridVisual::setNumMinorLinesPerMajorLine)
            .def(
                "getNumMinorLinesPerMajorLine",
                &gui::GridVisual::getNumMinorLinesPerMajorLine)
            .def("setPlaneType", &gui::GridVisual::setPlaneType)
            .def("getPlaneType", &gui::GridVisual::getPlaneType)
            .def("setOffset", &gui::GridVisual::setOffset)
            .def("getOffset", &gui::GridVisual::getOffset)
            .def("setMajorLineColor", &gui::GridVisual::setMajorLineColor)
            .def("getMajorLineColor", &gui::GridVisual::getMajorLineColor)
            .def("setMinorLineColor", &gui::GridVisual::setMinorLineColor)
            .def("getMinorLineColor", &gui::GridVisual::getMinorLineColor)
            .def("setAxisLineWidth", &gui::GridVisual::setAxisLineWidth)
            .def("getAxisLineWidth", &gui::GridVisual::getAxisLineWidth)
            .def("setMajorLineWidth", &gui::GridVisual::setMajorLineWidth)
            .def("getMajorLineWidth", &gui::GridVisual::getMajorLineWidth)
            .def("setMinorLineWidth", &gui::GridVisual::setMinorLineWidth)
            .def("getMinorLineWidth", &gui::GridVisual::getMinorLineWidth)
            .def("display", &gui::GridVisual::display, nb::arg("display"))
            .def("isDisplayed", &gui::GridVisual::isDisplayed)
            .def("refresh", &gui::GridVisual::refresh);
  nb::enum_<gui::GridVisual::PlaneType>(grid, "PlaneType")
      .value("XY", gui::GridVisual::PlaneType::XY)
      .value("YZ", gui::GridVisual::PlaneType::YZ)
      .value("ZX", gui::GridVisual::PlaneType::ZX);

  dartnb::dart_class<gui::SupportPolygonVisual, gui::ViewerAttachment>(
      m, "SupportPolygonVisual")
      .def(
          dartnb::init<const dynamics::SkeletonPtr&, double>(),
          nb::arg("skeleton").none() = nullptr,
          nb::arg("elevation") = 0.02)
      .def(
          dartnb::init<const dynamics::SkeletonPtr&, std::size_t, double>(),
          nb::arg("skeleton"),
          nb::arg("treeIndex"),
          nb::arg("elevation") = 0.02)
      .def(
          "visualizeWholeSkeleton",
          &gui::SupportPolygonVisual::visualizeWholeSkeleton)
      .def(
          "visualizeTree",
          &gui::SupportPolygonVisual::visualizeTree,
          nb::arg("treeIndex"))
      .def(
          "setSkeleton",
          &gui::SupportPolygonVisual::setSkeleton,
          nb::arg("skeleton").none())
      .def("getSkeleton", &gui::SupportPolygonVisual::getSkeleton)
      .def(
          "setDisplayElevation",
          &gui::SupportPolygonVisual::setDisplayElevation)
      .def(
          "getDisplayElevation",
          &gui::SupportPolygonVisual::getDisplayElevation)
      .def("setPolygonColor", &gui::SupportPolygonVisual::setPolygonColor)
      .def("getPolygonColor", &gui::SupportPolygonVisual::getPolygonColor)
      .def("setCentroidRadius", &gui::SupportPolygonVisual::setCentroidRadius)
      .def("getCentroidRadius", &gui::SupportPolygonVisual::getCentroidRadius)
      .def(
          "setCenterOfMassRadius",
          &gui::SupportPolygonVisual::setCenterOfMassRadius)
      .def(
          "getCenterOfMassRadius",
          &gui::SupportPolygonVisual::getCenterOfMassRadius)
      .def("setValidCOMColor", &gui::SupportPolygonVisual::setValidCOMColor)
      .def("getValidCOMColor", &gui::SupportPolygonVisual::getValidCOMColor)
      .def("setInvalidCOMColor", &gui::SupportPolygonVisual::setInvalidCOMColor)
      .def("getInvalidCOMColor", &gui::SupportPolygonVisual::getInvalidCOMColor)
      .def(
          "displayPolygon",
          &gui::SupportPolygonVisual::displayPolygon,
          nb::arg("display"))
      .def("isPolygonDisplayed", &gui::SupportPolygonVisual::isPolygonDisplayed)
      .def(
          "displayCentroid",
          &gui::SupportPolygonVisual::displayCentroid,
          nb::arg("display"))
      .def(
          "isCentroidDisplayed",
          &gui::SupportPolygonVisual::isCentroidDisplayed)
      .def(
          "displayCenterOfMass",
          &gui::SupportPolygonVisual::displayCenterOfMass,
          nb::arg("display"))
      .def(
          "isCenterOfMassDisplayed",
          &gui::SupportPolygonVisual::isCenterOfMassDisplayed)
      .def("refresh", &gui::SupportPolygonVisual::refresh);

  dartnb::dart_class<gui::DebugOverlay, gui::ViewerAttachment>(
      m, "DebugOverlay")
      .def(dartnb::init<>())
      .def("setFont", &gui::DebugOverlay::setFont)
      .def("getFont", &gui::DebugOverlay::getFont)
      .def("setLineWidth", &gui::DebugOverlay::setLineWidth)
      .def("getLineWidth", &gui::DebugOverlay::getLineWidth)
      .def("setCharacterSize", &gui::DebugOverlay::setCharacterSize)
      .def("getCharacterSize", &gui::DebugOverlay::getCharacterSize)
      .def("setTextOutlineWidth", &gui::DebugOverlay::setTextOutlineWidth)
      .def("getTextOutlineWidth", &gui::DebugOverlay::getTextOutlineWidth)
      .def("setTextOutlineColor", &gui::DebugOverlay::setTextOutlineColor)
      .def("getTextOutlineColor", &gui::DebugOverlay::getTextOutlineColor)
      .def(
          "addLine",
          &gui::DebugOverlay::addLine,
          nb::arg("start"),
          nb::arg("end"),
          nb::arg("color"))
      .def(
          "addLabel",
          nb::overload_cast<
              const Eigen::Vector3d&,
              const std::string&,
              const Eigen::Vector4d&>(&gui::DebugOverlay::addLabel),
          nb::arg("position"),
          nb::arg("text"),
          nb::arg("color"))
      .def(
          "addLabel",
          nb::overload_cast<
              const Eigen::Vector3d&,
              const std::string&,
              const Eigen::Vector4d&,
              double>(&gui::DebugOverlay::addLabel),
          nb::arg("position"),
          nb::arg("text"),
          nb::arg("color"),
          nb::arg("characterSize"))
      .def(
          "addScreenLabel",
          &gui::DebugOverlay::addScreenLabel,
          nb::arg("position"),
          nb::arg("text"),
          nb::arg("color"),
          nb::arg("size") = 16.0)
      .def(
          "addLabelPixels",
          &gui::DebugOverlay::addLabelPixels,
          nb::arg("position"),
          nb::arg("text"),
          nb::arg("color"),
          nb::arg("size") = 16.0)
      .def("clear", &gui::DebugOverlay::clear)
      .def("getNumLines", &gui::DebugOverlay::getNumLines)
      .def("getNumLabels", &gui::DebugOverlay::getNumLabels)
      .def("refresh", &gui::DebugOverlay::refresh);

  dartnb::dart_class<gui::Viewer, PyViewer>(m, "Viewer")
      .def(
          dartnb::init<const Eigen::Vector4d&>(),
          nb::arg("clearColor") = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0))
      .def(
          "addWorldNode",
          [](gui::Viewer& self,
             std::shared_ptr<gui::WorldNode> node,
             bool active) {
            dartnb::gui::retain(
                &self,
                node,
                nb::overload_cast<const std::shared_ptr<gui::WorldNode>&>(
                    &gui::Viewer::removeWorldNode));
            try {
              self.addWorldNode(node, active);
            } catch (...) {
              dartnb::gui::retire(&self, node.get());
              throw;
            }
          },
          nb::arg("node"),
          nb::arg("active") = true)
      .def(
          "removeWorldNode",
          [](gui::Viewer& self, const std::shared_ptr<gui::WorldNode>& node) {
            self.removeWorldNode(node);
            dartnb::gui::retire(&self, node.get());
          },
          nb::arg("node"))
      .def(
          "removeWorldNode",
          [](gui::Viewer& self,
             const std::shared_ptr<simulation::World>& world) {
            auto node = self.getWorldNode(world);
            self.removeWorldNode(world);
            dartnb::gui::retire(&self, node.get());
          },
          nb::arg("world"))
      .def("getWorldNode", &gui::Viewer::getWorldNode, nb::arg("world"))
      .def("getNumWorldNodes", &gui::Viewer::getNumWorldNodes)
      .def(
          "setWorldNodeActive",
          nb::overload_cast<const std::shared_ptr<gui::WorldNode>&, bool>(
              &gui::Viewer::setWorldNodeActive),
          nb::arg("node"),
          nb::arg("active") = true)
      .def(
          "setWorldNodeActive",
          nb::overload_cast<const std::shared_ptr<simulation::World>&, bool>(
              &gui::Viewer::setWorldNodeActive),
          nb::arg("world"),
          nb::arg("active") = true)
      .def(
          "addAttachment",
          [](gui::Viewer& self,
             std::shared_ptr<gui::ViewerAttachment> attachment) {
            dartnb::gui::retain(
                &self, attachment, &gui::Viewer::removeAttachment);
            try {
              self.addAttachment(attachment);
            } catch (...) {
              dartnb::gui::retire(&self, attachment.get());
              throw;
            }
          },
          nb::arg("attachment"))
      .def(
          "removeAttachment",
          [](gui::Viewer& self,
             const std::shared_ptr<gui::ViewerAttachment>& attachment) {
            try {
              self.removeAttachment(attachment);
            } catch (...) {
              dartnb::gui::retire(&self, attachment.get());
              throw;
            }
            dartnb::gui::retire(&self, attachment.get());
          },
          nb::arg("attachment"))
      .def("getAttachments", &gui::Viewer::getAttachments)
      .def(
          "addWidget",
          [](gui::Viewer& self, std::shared_ptr<gui::ImGuiWidget> widget) {
            self.addWidget(widget);
            dartnb::gui::retain(&self, widget, &gui::Viewer::removeWidget);
          },
          nb::arg("widget"))
      .def(
          "removeWidget",
          [](gui::Viewer& self,
             const std::shared_ptr<gui::ImGuiWidget>& widget) {
            self.removeWidget(widget);
            dartnb::gui::retire(&self, widget.get());
          },
          nb::arg("widget"))
      .def(
          "addEventHandler",
          [](gui::Viewer& self, std::shared_ptr<gui::GUIEventHandler> handler) {
            self.addEventHandler(handler);
            dartnb::gui::retain(
                &self, handler, &gui::Viewer::removeEventHandler);
          },
          nb::arg("handler"))
      .def(
          "removeEventHandler",
          [](gui::Viewer& self,
             const std::shared_ptr<gui::GUIEventHandler>& handler) {
            self.removeEventHandler(handler);
            dartnb::gui::retire(&self, handler.get());
          },
          nb::arg("handler"))
      .def("handleEvent", &gui::Viewer::handleEvent, nb::arg("event"))
      .def(
          "switchDefaultEventHandler",
          &gui::Viewer::switchDefaultEventHandler,
          nb::arg("on"))
      .def(
          "getDefaultEventHandler",
          &gui::Viewer::getDefaultEventHandler,
          nb::rv_policy::reference_internal)
      .def(
          "addInstructionText",
          &gui::Viewer::addInstructionText,
          nb::arg("text"))
      .def("getInstructions", &gui::Viewer::getInstructions)
      .def(
          "enableDragAndDrop",
          [dragOwners](gui::Viewer& self, dynamics::Entity* entity) {
            auto* interaction = self.enableDragAndDrop(entity);
            return dart_gui_owned_drag(
                interaction ? interaction->shared_from_this() : nullptr,
                dragOwners);
          },
          nb::arg("entity"))
      .def(
          "enableDragAndDrop",
          [dragOwners](gui::Viewer& self, dynamics::SimpleFrame* frame) {
            auto* interaction = self.enableDragAndDrop(frame);
            return dart_gui_owned_drag(
                interaction ? interaction->shared_from_this() : nullptr,
                dragOwners);
          },
          nb::arg("frame"))
      .def(
          "enableDragAndDrop",
          [dragOwners](
              gui::Viewer& self,
              dynamics::SimpleFrame* frame,
              dynamics::Shape* shape) {
            auto* interaction = self.enableDragAndDrop(frame, shape);
            return dart_gui_owned_drag(
                interaction ? interaction->shared_from_this() : nullptr,
                dragOwners);
          },
          nb::arg("frame"),
          nb::arg("shape"))
      .def(
          "enableDragAndDrop",
          [dragOwners](gui::Viewer& self, gui::InteractiveFrame* frame) {
            auto* interaction = self.enableDragAndDrop(frame);
            return dart_gui_owned_drag(
                interaction ? interaction->shared_from_this() : nullptr,
                dragOwners);
          },
          nb::arg("frame"))
      .def(
          "enableDragAndDrop",
          [dragOwners](
              gui::Viewer& self,
              dynamics::BodyNode* body,
              bool useExternalIK,
              bool useWholeBody) {
            auto* interaction
                = self.enableDragAndDrop(body, useExternalIK, useWholeBody);
            return dart_gui_owned_drag(
                interaction ? interaction->shared_from_this() : nullptr,
                dragOwners);
          },
          nb::arg("bn"),
          nb::arg("useExternalIK") = true,
          nb::arg("useWholeBody") = false)
      .def(
          "disableDragAndDrop",
          &gui::Viewer::disableDragAndDrop,
          nb::arg("interaction"))
      .def("updateDragAndDrops", &gui::Viewer::updateDragAndDrops)
      .def(
          "computeIntersections",
          [](gui::Viewer& self, const gui::GUIEventAdapter& event) {
            std::vector<gui::PickInfo> picks;
            self.computeIntersections(event, picks);
            return picks;
          },
          nb::arg("event"))
      .def(
          "getRayUnderCursor",
          [](const gui::Viewer& self, double x, double y) {
            Eigen::Vector3d near, far;
            self.getRayUnderCursor(x, y, near, far);
            return nb::make_tuple(near, far);
          },
          nb::arg("x"),
          nb::arg("y"))
      .def("getCameraDirection", &gui::Viewer::getCameraDirection)
      .def("setWireframe", &gui::Viewer::setWireframe, nb::arg("enabled"))
      .def("isWireframe", &gui::Viewer::isWireframe)
      .def(
          "setBodyWireframe",
          &gui::Viewer::setBodyWireframe,
          nb::arg("body"),
          nb::arg("enabled"))
      .def("getBodyWireframe", &gui::Viewer::getBodyWireframe, nb::arg("body"))
      .def("simulate", &gui::Viewer::simulate, nb::arg("on"))
      .def("isSimulating", &gui::Viewer::isSimulating)
      .def("allowSimulation", &gui::Viewer::allowSimulation, nb::arg("allow"))
      .def("isAllowingSimulation", &gui::Viewer::isAllowingSimulation)
      .def(
          "setCameraHome",
          &gui::Viewer::setCameraHome,
          nb::arg("eye"),
          nb::arg("center"),
          nb::arg("up"))
      .def(
          "setCameraHomePosition",
          &gui::Viewer::setCameraHomePosition,
          nb::arg("eye"),
          nb::arg("center"),
          nb::arg("up"))
      .def("home", &gui::Viewer::home)
      .def(
          "setVerticalFieldOfView",
          &gui::Viewer::setVerticalFieldOfView,
          nb::arg("fov"))
      .def("getVerticalFieldOfView", &gui::Viewer::getVerticalFieldOfView)
      .def(
          "setUpwardsDirection",
          &gui::Viewer::setUpwardsDirection,
          nb::arg("up"))
      .def("setCameraMode", &gui::Viewer::setCameraMode, nb::arg("mode"))
      .def("getCameraMode", &gui::Viewer::getCameraMode)
      .def("switchHeadlights", &gui::Viewer::switchHeadlights, nb::arg("on"))
      .def("checkHeadlights", &gui::Viewer::checkHeadlights)
      .def(
          "setUpViewInWindow",
          nb::overload_cast<int, int, int, int>(
              &gui::Viewer::setUpViewInWindow),
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("width"),
          nb::arg("height"))
      .def(
          "setUpViewInWindow",
          nb::overload_cast<>(&gui::Viewer::setUpViewInWindow))
      .def(
          "setUpViewForCapture",
          &gui::Viewer::setUpViewForCapture,
          nb::arg("width"),
          nb::arg("height"))
      .def("setGuiScale", &gui::Viewer::setGuiScale, nb::arg("scale"))
      .def("getGuiScale", &gui::Viewer::getGuiScale)
      .def(
          "setAutomaticGuiScale",
          &gui::Viewer::setAutomaticGuiScale,
          nb::arg("automatic") = true)
      .def(
          "setBackgroundColor",
          &gui::Viewer::setBackgroundColor,
          nb::arg("color"))
      .def("setWindowTitle", &gui::Viewer::setWindowTitle, nb::arg("title"))
      .def("captureScreen", &gui::Viewer::captureScreen, nb::arg("filename"))
      .def(
          "record",
          &gui::Viewer::record,
          nb::arg("directory"),
          nb::arg("prefix") = "image",
          nb::arg("restart") = false,
          nb::arg("digits") = 6)
      .def("pauseRecording", &gui::Viewer::pauseRecording)
      .def("isRecording", &gui::Viewer::isRecording)
      .def("updateViewer", &gui::Viewer::updateViewer)
      .def("frame", &gui::Viewer::frame)
      .def("run", &gui::Viewer::run)
      .def("setDone", &gui::Viewer::setDone, nb::arg("done") = true)
      .def("done", &gui::Viewer::done)
      .def("getRenderBackend", &gui::Viewer::getRenderBackend)
      .def("getRenderDevice", &gui::Viewer::getRenderDevice);
}

} // namespace python
} // namespace dart
