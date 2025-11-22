#include "gui/osg/osg.hpp"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/gui/osg/All.hpp>
#include <dart/gui/osg/DragAndDrop.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defDragAndDrop(nb::module_& m)
{
  using dart::gui::osg::DragAndDrop;

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
          nb::arg("rotationModKey"))
      .def("getRotationModKey", &DragAndDrop::getRotationModKey);

  auto attr = m.attr("DragAndDrop");

  nb::enum_<DragAndDrop::RotationOption>(attr, "RotationOption")
      .value("HOLD_MODKEY", DragAndDrop::RotationOption::HOLD_MODKEY)
      .value("ALWAYS_ON", DragAndDrop::RotationOption::ALWAYS_ON)
      .value("ALWAYS_OFF", DragAndDrop::RotationOption::ALWAYS_OFF);

  nb::class_<dart::gui::osg::SimpleFrameDnD, DragAndDrop>(m, "SimpleFrameDnD")
      .def(
          nb::init<dart::gui::osg::Viewer*, dart::dynamics::SimpleFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"))
      .def("move", &dart::gui::osg::SimpleFrameDnD::move)
      .def("saveState", &dart::gui::osg::SimpleFrameDnD::saveState);

  nb::class_<dart::gui::osg::SimpleFrameShapeDnD, dart::gui::osg::SimpleFrameDnD>(
      m, "SimpleFrameShapeDnD")
      .def(
          nb::init<
              dart::gui::osg::Viewer*,
              dart::dynamics::SimpleFrame*,
              dart::dynamics::Shape*>(),
          nb::arg("viewer"),
          nb::arg("frame"),
          nb::arg("shape"))
      .def("update", &dart::gui::osg::SimpleFrameShapeDnD::update);

  nb::class_<dart::gui::osg::BodyNodeDnD, DragAndDrop>(m, "BodyNodeDnD")
      .def(
          nb::init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*>(),
          nb::arg("viewer"),
          nb::arg("bn"))
      .def(
          nb::init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*, bool>(),
          nb::arg("viewer"),
          nb::arg("bn"),
          nb::arg("useExternalIK"))
      .def(
          nb::init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*, bool, bool>(),
          nb::arg("viewer"),
          nb::arg("bn"),
          nb::arg("useExternalIK"),
          nb::arg("useWholeBody"))
      .def("update", &dart::gui::osg::BodyNodeDnD::update)
      .def("move", &dart::gui::osg::BodyNodeDnD::move)
      .def("saveState", &dart::gui::osg::BodyNodeDnD::saveState)
      .def("release", &dart::gui::osg::BodyNodeDnD::release)
      .def(
          "useExternalIK",
          [](dart::gui::osg::BodyNodeDnD& self, bool external) {
            self.useExternalIK(external);
          },
          nb::arg("external"))
      .def("isUsingExternalIK", &dart::gui::osg::BodyNodeDnD::isUsingExternalIK)
      .def(
          "useWholeBody",
          [](dart::gui::osg::BodyNodeDnD& self, bool wholeBody) {
            self.useWholeBody(wholeBody);
          },
          nb::arg("wholeBody"))
      .def("isUsingWholeBody", &dart::gui::osg::BodyNodeDnD::isUsingWholeBody)
      .def(
          "setPreserveOrientationModKey",
          [](dart::gui::osg::BodyNodeDnD& self, osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self.setPreserveOrientationModKey(modkey);
          },
          nb::arg("modkey"))
      .def("getPreserveOrientationModKey", &dart::gui::osg::BodyNodeDnD::getPreserveOrientationModKey)
      .def(
          "setJointRestrictionModKey",
          [](dart::gui::osg::BodyNodeDnD& self, osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self.setJointRestrictionModKey(modkey);
          },
          nb::arg("modkey"))
      .def("getJointRestrictionModKey", &dart::gui::osg::BodyNodeDnD::getJointRestrictionModKey);

  nb::class_<dart::gui::osg::InteractiveFrameDnD, DragAndDrop>(m, "InteractiveFrameDnD")
      .def(
          nb::init<dart::gui::osg::Viewer*, dart::gui::osg::InteractiveFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"))
      .def("update", &dart::gui::osg::InteractiveFrameDnD::update)
      .def("move", &dart::gui::osg::InteractiveFrameDnD::move)
      .def("saveState", &dart::gui::osg::InteractiveFrameDnD::saveState);
}

} // namespace dart::python_nb
