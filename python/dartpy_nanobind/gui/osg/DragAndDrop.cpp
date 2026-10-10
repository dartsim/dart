#include "detail/dart_nb.hpp"
#include "detail/eigen.hpp"

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
#include <dart/gui/osg/Viewer.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/common/Observer.hpp>
#include <dart/common/Subject.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <osgGA/GUIEventAdapter>

#include <memory>

namespace dart {
namespace python {

void DragAndDrop(nb::module_& m)
{
  dartnb::dart_class<
      dart::gui::osg::DragAndDrop,
      dart::common::Observer,
      dart::common::Subject>(m, "DragAndDrop")
      .def(
          "update", +[](dart::gui::osg::DragAndDrop* self) { self->update(); })
      .def(
          "setObstructable",
          +[](dart::gui::osg::DragAndDrop* self, bool _obstructable) {
            self->setObstructable(_obstructable);
          },
          nb::arg("obstructable"))
      .def(
          "isObstructable",
          +[](const dart::gui::osg::DragAndDrop* self) -> bool {
            return self->isObstructable();
          })
      .def(
          "move", +[](dart::gui::osg::DragAndDrop* self) { self->move(); })
      .def(
          "saveState",
          +[](dart::gui::osg::DragAndDrop* self) { self->saveState(); })
      .def(
          "release",
          +[](dart::gui::osg::DragAndDrop* self) { self->release(); })
      .def(
          "getConstrainedDx",
          +[](const dart::gui::osg::DragAndDrop* self) -> Eigen::Vector3d {
            return self->getConstrainedDx();
          })
      .def(
          "getConstrainedRotation",
          +[](const dart::gui::osg::DragAndDrop* self) -> Eigen::AngleAxisd {
            return self->getConstrainedRotation();
          })
      .def(
          "unconstrain",
          +[](dart::gui::osg::DragAndDrop* self) { self->unconstrain(); })
      .def(
          "constrainToLine",
          +[](dart::gui::osg::DragAndDrop* self, const Eigen::Vector3d& slope) {
            self->constrainToLine(slope);
          },
          nb::arg("slope"))
      .def(
          "constrainToPlane",
          +[](dart::gui::osg::DragAndDrop* self,
              const Eigen::Vector3d& normal) {
            self->constrainToPlane(normal);
          },
          nb::arg("normal"))
      .def(
          "isMoving",
          +[](const dart::gui::osg::DragAndDrop* self) -> bool {
            return self->isMoving();
          })
      .def(
          "setRotationOption",
          +[](dart::gui::osg::DragAndDrop* self,
              dart::gui::osg::DragAndDrop::RotationOption option) {
            self->setRotationOption(option);
          },
          nb::arg("option"))
      .def(
          "getRotationOption",
          +[](const dart::gui::osg::DragAndDrop* self)
              -> dart::gui::osg::DragAndDrop::RotationOption {
            return self->getRotationOption();
          })
      .def(
          "setRotationModKey",
          +[](dart::gui::osg::DragAndDrop* self,
              osgGA::GUIEventAdapter::ModKeyMask rotationModKey) {
            self->setRotationModKey(rotationModKey);
          },
          nb::arg("rotationModKey"))
      .def(
          "getRotationModKey",
          +[](const dart::gui::osg::DragAndDrop* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getRotationModKey();
          });

  auto attr = m.attr("DragAndDrop");

  nb::enum_<dart::gui::osg::DragAndDrop::RotationOption>(
      attr, "RotationOption", nb::is_arithmetic())
      .value(
          "HOLD_MODKEY",
          dart::gui::osg::DragAndDrop::RotationOption::HOLD_MODKEY)
      .value(
          "ALWAYS_ON", dart::gui::osg::DragAndDrop::RotationOption::ALWAYS_ON)
      .value(
          "ALWAYS_OFF", dart::gui::osg::DragAndDrop::RotationOption::ALWAYS_OFF)
      .export_values();

  dartnb::dart_class<
      dart::gui::osg::SimpleFrameDnD,
      dart::gui::osg::DragAndDrop>(m, "SimpleFrameDnD")
      .def(
          dartnb::gui::
              dnd_init<dart::gui::osg::Viewer*, dart::dynamics::SimpleFrame*>(),
          nb::arg("viewer").none(),
          nb::arg("frame").none())
      .def(
          "move", +[](dart::gui::osg::SimpleFrameDnD* self) { self->move(); })
      .def(
          "saveState",
          +[](dart::gui::osg::SimpleFrameDnD* self) { self->saveState(); });

  dartnb::dart_class<
      dart::gui::osg::SimpleFrameShapeDnD,
      dart::gui::osg::SimpleFrameDnD>(m, "SimpleFrameShapeDnD")
      .def(
          dartnb::gui::dnd_init<
              dart::gui::osg::Viewer*,
              dart::dynamics::SimpleFrame*,
              dart::dynamics::Shape*>(),
          nb::arg("viewer").none(),
          nb::arg("frame").none(),
          nb::arg("shape").none())
      .def(
          "update",
          +[](dart::gui::osg::SimpleFrameShapeDnD* self) { self->update(); });

  dartnb::dart_class<dart::gui::osg::BodyNodeDnD, dart::gui::osg::DragAndDrop>(
      m, "BodyNodeDnD")
      .def(
          dartnb::gui::
              dnd_init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*>(),
          nb::arg("viewer").none(),
          nb::arg("bn").none())
      .def(
          dartnb::gui::dnd_init<
              dart::gui::osg::Viewer*,
              dart::dynamics::BodyNode*,
              bool>(),
          nb::arg("viewer").none(),
          nb::arg("bn").none(),
          nb::arg("useExternalIK"))
      .def(
          dartnb::gui::dnd_init<
              dart::gui::osg::Viewer*,
              dart::dynamics::BodyNode*,
              bool,
              bool>(),
          nb::arg("viewer").none(),
          nb::arg("bn").none(),
          nb::arg("useExternalIK"),
          nb::arg("useWholeBody"))
      .def(
          "update", +[](dart::gui::osg::BodyNodeDnD* self) { self->update(); })
      .def(
          "move", +[](dart::gui::osg::BodyNodeDnD* self) { self->move(); })
      .def(
          "saveState",
          +[](dart::gui::osg::BodyNodeDnD* self) { self->saveState(); })
      .def(
          "release",
          +[](dart::gui::osg::BodyNodeDnD* self) { self->release(); })
      .def(
          "useExternalIK",
          +[](dart::gui::osg::BodyNodeDnD* self, bool external) {
            self->useExternalIK(external);
          },
          nb::arg("external"))
      .def(
          "isUsingExternalIK",
          +[](const dart::gui::osg::BodyNodeDnD* self) -> bool {
            return self->isUsingExternalIK();
          })
      .def(
          "useWholeBody",
          +[](dart::gui::osg::BodyNodeDnD* self, bool wholeBody) {
            self->useWholeBody(wholeBody);
          },
          nb::arg("wholeBody"))
      .def(
          "isUsingWholeBody",
          +[](const dart::gui::osg::BodyNodeDnD* self) -> bool {
            return self->isUsingWholeBody();
          })
      .def(
          "setPreserveOrientationModKey",
          +[](dart::gui::osg::BodyNodeDnD* self,
              osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self->setPreserveOrientationModKey(modkey);
          },
          nb::arg("modkey"))
      .def(
          "getPreserveOrientationModKey",
          +[](const dart::gui::osg::BodyNodeDnD* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getPreserveOrientationModKey();
          })
      .def(
          "setJointRestrictionModKey",
          +[](dart::gui::osg::BodyNodeDnD* self,
              osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self->setJointRestrictionModKey(modkey);
          },
          nb::arg("modkey"))
      .def(
          "getJointRestrictionModKey",
          +[](const dart::gui::osg::BodyNodeDnD* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getJointRestrictionModKey();
          });

  dartnb::dart_class<
      dart::gui::osg::InteractiveFrameDnD,
      dart::gui::osg::DragAndDrop>(m, "InteractiveFrameDnD")
      .def(
          dartnb::gui::dnd_init<
              dart::gui::osg::Viewer*,
              dart::gui::osg::InteractiveFrame*>(),
          nb::arg("viewer").none(),
          nb::arg("frame").none())
      .def(
          "update",
          +[](dart::gui::osg::InteractiveFrameDnD* self) { self->update(); })
      .def(
          "move",
          +[](dart::gui::osg::InteractiveFrameDnD* self) { self->move(); })
      .def(
          "saveState", +[](dart::gui::osg::InteractiveFrameDnD* self) {
            self->saveState();
          });
}

} // namespace python
} // namespace dart
