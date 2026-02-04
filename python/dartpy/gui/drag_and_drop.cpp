#include "common/type_casters.hpp"
#include "gui/gui.hpp"

#include <dart/gui/drag_and_drop.hpp>
#include <dart/gui/interactive_frame.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defDragAndDrop(nb::module_& m)
{
  using dart::gui::DragAndDrop;

  nb::class_<DragAndDrop>(m, "DragAndDrop")
      .def("update", &DragAndDrop::update)
      .def(
          "setObstructable",
          [](DragAndDrop& self, bool obstructable) {
            self.setObstructable(obstructable);
          },
          nb::arg("obstructable"))
      .def("isObstructable", &DragAndDrop::isObstructable)
      .def("move", &DragAndDrop::move)
      .def("saveState", &DragAndDrop::saveState)
      .def("release", &DragAndDrop::release)
      .def("getConstrainedDx", &DragAndDrop::getConstrainedDx)
      .def("getConstrainedRotation", &DragAndDrop::getConstrainedRotation)
      .def("unconstrain", &DragAndDrop::unconstrain)
      .def(
          "constrainToLine",
          [](DragAndDrop& self, const Eigen::Vector3d& slope) {
            self.constrainToLine(slope);
          },
          nb::arg("slope"))
      .def(
          "constrainToPlane",
          [](DragAndDrop& self, const Eigen::Vector3d& normal) {
            self.constrainToPlane(normal);
          },
          nb::arg("normal"))
      .def("isMoving", &DragAndDrop::isMoving)
      .def(
          "setRotationOption",
          [](DragAndDrop& self, DragAndDrop::RotationOption option) {
            self.setRotationOption(option);
          },
          nb::arg("option"))
      .def("getRotationOption", &DragAndDrop::getRotationOption)
      .def(
          "setRotationModKey",
          [](DragAndDrop& self, osgGA::GUIEventAdapter::ModKeyMask modKey) {
            self.setRotationModKey(modKey);
          },
          nb::arg("rotation_mod_key"))
      .def("getRotationModKey", &DragAndDrop::getRotationModKey);

  auto attr = m.attr("DragAndDrop");

  nb::enum_<DragAndDrop::RotationOption>(attr, "RotationOption")
      .value("HOLD_MODKEY", DragAndDrop::RotationOption::HOLD_MODKEY)
      .value("ALWAYS_ON", DragAndDrop::RotationOption::ALWAYS_ON)
      .value("ALWAYS_OFF", DragAndDrop::RotationOption::ALWAYS_OFF);

  nb::class_<dart::gui::SimpleFrameDnD, DragAndDrop>(m, "SimpleFrameDnD")
      .def(
          nb::init<dart::gui::Viewer*, dart::dynamics::SimpleFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"))
      .def("move", &dart::gui::SimpleFrameDnD::move)
      .def("saveState", &dart::gui::SimpleFrameDnD::saveState);

  nb::class_<dart::gui::SimpleFrameShapeDnD, dart::gui::SimpleFrameDnD>(
      m, "SimpleFrameShapeDnD")
      .def(
          nb::init<
              dart::gui::Viewer*,
              dart::dynamics::SimpleFrame*,
              dart::dynamics::Shape*>(),
          nb::arg("viewer"),
          nb::arg("frame"),
          nb::arg("shape"))
      .def("update", &dart::gui::SimpleFrameShapeDnD::update);

  nb::class_<dart::gui::BodyNodeDnD, DragAndDrop>(m, "BodyNodeDnD")
      .def(
          nb::init<dart::gui::Viewer*, dart::dynamics::BodyNode*>(),
          nb::arg("viewer"),
          nb::arg("bn"))
      .def(
          nb::init<dart::gui::Viewer*, dart::dynamics::BodyNode*, bool>(),
          nb::arg("viewer"),
          nb::arg("bn"),
          nb::arg("use_external_ik"))
      .def(
          nb::init<dart::gui::Viewer*, dart::dynamics::BodyNode*, bool, bool>(),
          nb::arg("viewer"),
          nb::arg("bn"),
          nb::arg("use_external_ik"),
          nb::arg("use_whole_body"))
      .def("update", &dart::gui::BodyNodeDnD::update)
      .def("move", &dart::gui::BodyNodeDnD::move)
      .def("saveState", &dart::gui::BodyNodeDnD::saveState)
      .def("release", &dart::gui::BodyNodeDnD::release)
      .def(
          "useExternalIK",
          [](dart::gui::BodyNodeDnD& self, bool external) {
            self.useExternalIK(external);
          },
          nb::arg("external"))
      .def("isUsingExternalIK", &dart::gui::BodyNodeDnD::isUsingExternalIK)
      .def(
          "useWholeBody",
          [](dart::gui::BodyNodeDnD& self, bool wholeBody) {
            self.useWholeBody(wholeBody);
          },
          nb::arg("whole_body"))
      .def("isUsingWholeBody", &dart::gui::BodyNodeDnD::isUsingWholeBody)
      .def(
          "setPreserveOrientationModKey",
          [](dart::gui::BodyNodeDnD& self,
             osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self.setPreserveOrientationModKey(modkey);
          },
          nb::arg("modkey"))
      .def(
          "getPreserveOrientationModKey",
          &dart::gui::BodyNodeDnD::getPreserveOrientationModKey)
      .def(
          "setJointRestrictionModKey",
          [](dart::gui::BodyNodeDnD& self,
             osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self.setJointRestrictionModKey(modkey);
          },
          nb::arg("modkey"))
      .def(
          "getJointRestrictionModKey",
          &dart::gui::BodyNodeDnD::getJointRestrictionModKey);

  nb::class_<dart::gui::InteractiveFrameDnD, DragAndDrop>(
      m, "InteractiveFrameDnD")
      .def(
          nb::init<dart::gui::Viewer*, dart::gui::InteractiveFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"))
      .def("update", &dart::gui::InteractiveFrameDnD::update)
      .def("move", &dart::gui::InteractiveFrameDnD::move)
      .def("saveState", &dart::gui::InteractiveFrameDnD::saveState);
}

} // namespace dart::python_nb
