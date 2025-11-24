#include "common/type_casters.hpp"
#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Entity.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

Eigen::Vector4d toVec4(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector4d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 4)
      throw nb::type_error("Expected a length-4 sequence");
    Eigen::Vector4d vec;
    for (ssize_t i = 0; i < 4; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

Eigen::Vector3d toVec3(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector3d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 3)
      throw nb::type_error("Expected a length-3 sequence");
    Eigen::Vector3d vec;
    for (ssize_t i = 0; i < 3; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

} // namespace

void defViewer(nb::module_& m)
{
  using dart::gui::Viewer;

  nb::class_<osgViewer::View>(m, "osgViewer")
      .def(nb::new_([]() { return makeOsgShared<osgViewer::View>(); }))
      .def(
          "addEventHandler",
          [](osgViewer::View& self, osgGA::GUIEventHandler* eventHandler) {
            self.addEventHandler(eventHandler);
          },
          nb::arg("eventHandler"));

  auto viewer
      = nb::class_<Viewer, osgViewer::View>(m, "Viewer")
            .def(nb::new_([]() { return makeOsgShared<Viewer>(); }))
            .def(
                nb::new_([](const nb::handle& color) {
                  Eigen::Vector4d c = toVec4(color);
                  return makeOsgShared<Viewer>(
                      ::osg::Vec4(c[0], c[1], c[2], c[3]));
                }),
                nb::arg("clearColor"))
            .def(
                nb::new_([](const osg::Vec4& clearColor) {
                  return makeOsgShared<Viewer>(clearColor);
                }),
                nb::arg("clearColor"))
            .def(
                "captureScreen",
                [](Viewer& self, const std::string& filename) {
                  self.captureScreen(filename);
                },
                nb::arg("filename"))
            .def(
                "record",
                [](Viewer& self, const std::string& directory) {
                  self.record(directory);
                },
                nb::arg("directory"))
            .def(
                "record",
                [](Viewer& self,
                   const std::string& directory,
                   const std::string& prefix) {
                  self.record(directory, prefix);
                },
                nb::arg("directory"),
                nb::arg("prefix"))
            .def(
                "record",
                [](Viewer& self,
                   const std::string& directory,
                   const std::string& prefix,
                   bool restart) { self.record(directory, prefix, restart); },
                nb::arg("directory"),
                nb::arg("prefix"),
                nb::arg("restart"))
            .def(
                "record",
                [](Viewer& self,
                   const std::string& directory,
                   const std::string& prefix,
                   bool restart,
                   std::size_t digits) {
                  self.record(directory, prefix, restart, digits);
                },
                nb::arg("directory"),
                nb::arg("prefix"),
                nb::arg("restart"),
                nb::arg("digits"))
            .def("pauseRecording", &Viewer::pauseRecording)
            .def("isRecording", &Viewer::isRecording)
            .def(
                "switchDefaultEventHandler",
                [](Viewer& self, bool on) {
                  self.switchDefaultEventHandler(on);
                },
                nb::arg("on"))
            .def(
                "switchHeadlights",
                [](Viewer& self, bool on) { self.switchHeadlights(on); },
                nb::arg("on"))
            .def("checkHeadlights", &Viewer::checkHeadlights)
            .def(
                "setLightingMode",
                &Viewer::setLightingMode,
                nb::arg("lightingMode"))
            .def("getLightingMode", &Viewer::getLightingMode)
            .def(
                "addWorldNode",
                [](Viewer& self, dart::gui::WorldNode* node) {
                  self.addWorldNode(node);
                },
                nb::arg("newWorldNode"))
            .def(
                "addWorldNode",
                [](Viewer& self, dart::gui::WorldNode* node, bool active) {
                  self.addWorldNode(node, active);
                },
                nb::arg("newWorldNode"),
                nb::arg("active"))
            .def(
                "removeWorldNode",
                [](Viewer& self, dart::gui::WorldNode* node) {
                  self.removeWorldNode(node);
                },
                nb::arg("oldWorldNode"))
            .def(
                "removeWorldNode",
                [](Viewer& self,
                   std::shared_ptr<dart::simulation::World> world) {
                  self.removeWorldNode(std::move(world));
                },
                nb::arg("oldWorld"))
            .def(
                "addAttachment",
                [](Viewer& self, dart::gui::ViewerAttachment* attachment) {
                  self.addAttachment(attachment);
                },
                nb::arg("attachment"))
            .def(
                "removeAttachment",
                [](Viewer& self, dart::gui::ViewerAttachment* attachment) {
                  self.removeAttachment(attachment);
                },
                nb::arg("attachment"))
            .def("setupDefaultLights", &Viewer::setupDefaultLights)
            .def(
                "setUpwardsDirection",
                [](Viewer& self, const osg::Vec3& up) {
                  self.setUpwardsDirection(up);
                },
                nb::arg("up"))
            .def(
                "setUpwardsDirection",
                [](Viewer& self, const Eigen::Vector3d& up) {
                  self.setUpwardsDirection(up);
                },
                nb::arg("up"))
            .def(
                "setWorldNodeActive",
                [](Viewer& self, dart::gui::WorldNode* node) {
                  self.setWorldNodeActive(node);
                },
                nb::arg("node"))
            .def(
                "setWorldNodeActive",
                [](Viewer& self, dart::gui::WorldNode* node, bool active) {
                  self.setWorldNodeActive(node, active);
                },
                nb::arg("node"),
                nb::arg("active"))
            .def(
                "setWorldNodeActive",
                [](Viewer& self,
                   std::shared_ptr<dart::simulation::World> world) {
                  self.setWorldNodeActive(std::move(world));
                },
                nb::arg("world"))
            .def(
                "setWorldNodeActive",
                [](Viewer& self,
                   std::shared_ptr<dart::simulation::World> world,
                   bool active) {
                  self.setWorldNodeActive(std::move(world), active);
                },
                nb::arg("world"),
                nb::arg("active"))
            .def("simulate", &Viewer::simulate, nb::arg("on"))
            .def("isSimulating", &Viewer::isSimulating)
            .def("allowSimulation", &Viewer::allowSimulation, nb::arg("allow"))
            .def("isAllowingSimulation", &Viewer::isAllowingSimulation)
            .def(
                "enableDragAndDrop",
                [](Viewer& self, dart::gui::InteractiveFrame* frame) {
                  return self.enableDragAndDrop(frame);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"))
            .def(
                "enableDragAndDrop",
                [](Viewer& self, dart::dynamics::SimpleFrame* frame) {
                  return self.enableDragAndDrop(frame);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"))
            .def(
                "enableDragAndDrop",
                [](Viewer& self,
                   dart::dynamics::SimpleFrame* frame,
                   dart::dynamics::Shape* shape) {
                  return self.enableDragAndDrop(frame, shape);
                },
                nb::rv_policy::reference_internal,
                nb::arg("frame"),
                nb::arg("shape"))
            .def(
                "enableDragAndDrop",
                [](Viewer& self,
                   dart::dynamics::BodyNode* bn,
                   bool useExternalIK,
                   bool useWholeBody) {
                  return self.enableDragAndDrop(
                      bn, useExternalIK, useWholeBody);
                },
                nb::rv_policy::reference_internal,
                nb::arg("bodyNode"),
                nb::arg("useExternalIK") = true,
                nb::arg("useWholeBody") = false)
            .def(
                "enableDragAndDrop",
                [](Viewer& self, dart::dynamics::Entity* entity) {
                  return self.enableDragAndDrop(entity);
                },
                nb::rv_policy::reference_internal,
                nb::arg("entity"))
            .def(
                "disableDragAndDrop",
                [](Viewer& self, dart::gui::InteractiveFrameDnD* dnd) {
                  self.disableDragAndDrop(dnd);
                },
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                [](Viewer& self, dart::gui::SimpleFrameDnD* dnd) {
                  self.disableDragAndDrop(dnd);
                },
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                [](Viewer& self, dart::gui::SimpleFrameShapeDnD* dnd) {
                  self.disableDragAndDrop(dnd);
                },
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                [](Viewer& self, dart::gui::BodyNodeDnD* dnd) {
                  self.disableDragAndDrop(dnd);
                },
                nb::arg("dnd"))
            .def(
                "disableDragAndDrop",
                [](Viewer& self, dart::gui::DragAndDrop* dnd) {
                  self.disableDragAndDrop(dnd);
                },
                nb::arg("dnd"))
            .def(
                "getInstructions",
                [](const Viewer& self) -> const std::string& {
                  return self.getInstructions();
                },
                nb::rv_policy::reference_internal)
            .def(
                "addInstructionText",
                [](Viewer& self, const std::string& instruction) {
                  self.addInstructionText(instruction);
                },
                nb::arg("instruction"))
            .def("updateViewer", &Viewer::updateViewer)
            .def("updateDragAndDrops", &Viewer::updateDragAndDrops)
            .def(
                "setVerticalFieldOfView",
                [](Viewer& self, double fov) {
                  self.setVerticalFieldOfView(fov);
                },
                nb::arg("fov"))
            .def("getVerticalFieldOfView", &Viewer::getVerticalFieldOfView)
            .def("run", [](Viewer& self) { return self.run(); })
            .def("frame", [](Viewer& self) { self.frame(); })
            .def(
                "frame",
                [](Viewer& self, double simulationTime) {
                  self.frame(simulationTime);
                },
                nb::arg("simulationTime"))
            .def(
                "setUpViewInWindow",
                [](Viewer& self, int x, int y, int width, int height) {
                  self.setUpViewInWindow(x, y, width, height);
                },
                nb::arg("x"),
                nb::arg("y"),
                nb::arg("width"),
                nb::arg("height"))
            .def(
                "setCameraHomePosition",
                [](Viewer& self,
                   const nb::handle& eye,
                   const nb::handle& center,
                   const nb::handle& up) {
                  Eigen::Vector3d e = toVec3(eye);
                  Eigen::Vector3d c = toVec3(center);
                  Eigen::Vector3d u = toVec3(up);
                  auto* manip = self.getCameraManipulator();
                  if (manip) {
                    manip->setHomePosition(
                        dart::gui::eigToOsgVec3(e),
                        dart::gui::eigToOsgVec3(c),
                        dart::gui::eigToOsgVec3(u));
                    self.setCameraManipulator(manip);
                  }
                },
                nb::arg("eye"),
                nb::arg("center"),
                nb::arg("up"))
            .def("setCameraMode", &Viewer::setCameraMode, nb::arg("mode"))
            .def("getCameraMode", &Viewer::getCameraMode);

  nb::enum_<Viewer::LightingMode>(viewer, "LightingMode")
      .value("NO_LIGHT", Viewer::NO_LIGHT)
      .value("HEADLIGHT", Viewer::HEADLIGHT)
      .value("SKY_LIGHT", Viewer::SKY_LIGHT);

  nb::enum_<dart::gui::CameraMode>(m, "CameraMode")
      .value("RGBA", dart::gui::CameraMode::RGBA)
      .value("DEPTH", dart::gui::CameraMode::DEPTH);
}

} // namespace dart::python_nb
