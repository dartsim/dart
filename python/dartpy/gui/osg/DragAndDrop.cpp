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

void DragAndDrop(py::module& m)
{
  ::py::class_<
      dart::gui::DragAndDrop,
      dart::common::Observer,
      dart::common::Subject,
      ::std::shared_ptr<dart::gui::DragAndDrop>>(m, "DragAndDrop")
      .def(
          "update", +[](dart::gui::DragAndDrop* self) { self->update(); })
      .def(
          "setObstructable",
          +[](dart::gui::DragAndDrop* self, bool _obstructable) {
            self->setObstructable(_obstructable);
          },
          ::py::arg("obstructable"))
      .def(
          "isObstructable",
          +[](const dart::gui::DragAndDrop* self) -> bool {
            return self->isObstructable();
          })
      .def(
          "move", +[](dart::gui::DragAndDrop* self) { self->move(); })
      .def(
          "saveState", +[](dart::gui::DragAndDrop* self) { self->saveState(); })
      .def(
          "release", +[](dart::gui::DragAndDrop* self) { self->release(); })
      .def(
          "getConstrainedDx",
          +[](const dart::gui::DragAndDrop* self) -> Eigen::Vector3d {
            return self->getConstrainedDx();
          })
      .def(
          "getConstrainedRotation",
          +[](const dart::gui::DragAndDrop* self) -> Eigen::AngleAxisd {
            return self->getConstrainedRotation();
          })
      .def(
          "unconstrain",
          +[](dart::gui::DragAndDrop* self) { self->unconstrain(); })
      .def(
          "constrainToLine",
          +[](dart::gui::DragAndDrop* self, const Eigen::Vector3d& slope) {
            self->constrainToLine(slope);
          },
          ::py::arg("slope"))
      .def(
          "constrainToPlane",
          +[](dart::gui::DragAndDrop* self, const Eigen::Vector3d& normal) {
            self->constrainToPlane(normal);
          },
          ::py::arg("normal"))
      .def(
          "isMoving",
          +[](const dart::gui::DragAndDrop* self) -> bool {
            return self->isMoving();
          })
      .def(
          "setRotationOption",
          +[](dart::gui::DragAndDrop* self,
              dart::gui::DragAndDrop::RotationOption option) {
            self->setRotationOption(option);
          },
          ::py::arg("option"))
      .def(
          "getRotationOption",
          +[](const dart::gui::DragAndDrop* self)
              -> dart::gui::DragAndDrop::RotationOption {
            return self->getRotationOption();
          })
      .def(
          "setRotationModKey",
          +[](dart::gui::DragAndDrop* self,
              osgGA::GUIEventAdapter::ModKeyMask rotationModKey) {
            self->setRotationModKey(rotationModKey);
          },
          ::py::arg("rotationModKey"))
      .def(
          "getRotationModKey",
          +[](const dart::gui::DragAndDrop* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getRotationModKey();
          });

  auto attr = m.attr("DragAndDrop");

  ::py::enum_<dart::gui::DragAndDrop::RotationOption>(attr, "RotationOption")
      .value("HOLD_MODKEY", dart::gui::DragAndDrop::RotationOption::HOLD_MODKEY)
      .value("ALWAYS_ON", dart::gui::DragAndDrop::RotationOption::ALWAYS_ON)
      .value("ALWAYS_OFF", dart::gui::DragAndDrop::RotationOption::ALWAYS_OFF)
      .export_values();

  ::py::class_<
      dart::gui::SimpleFrameDnD,
      dart::gui::DragAndDrop,
      ::std::shared_ptr<dart::gui::SimpleFrameDnD>>(m, "SimpleFrameDnD")
      .def(
          ::py::init<dart::gui::Viewer*, dart::dynamics::SimpleFrame*>(),
          ::py::arg("viewer"),
          ::py::arg("frame"))
      .def(
          "move", +[](dart::gui::SimpleFrameDnD* self) { self->move(); })
      .def(
          "saveState",
          +[](dart::gui::SimpleFrameDnD* self) { self->saveState(); });

  ::py::class_<
      dart::gui::SimpleFrameShapeDnD,
      dart::gui::SimpleFrameDnD,
      std::shared_ptr<dart::gui::SimpleFrameShapeDnD>>(m, "SimpleFrameShapeDnD")
      .def(
          ::py::init<
              dart::gui::Viewer*,
              dart::dynamics::SimpleFrame*,
              dart::dynamics::Shape*>(),
          ::py::arg("viewer"),
          ::py::arg("frame"),
          ::py::arg("shape"))
      .def(
          "update",
          +[](dart::gui::SimpleFrameShapeDnD* self) { self->update(); });

  ::py::class_<
      dart::gui::BodyNodeDnD,
      dart::gui::DragAndDrop,
      std::shared_ptr<dart::gui::BodyNodeDnD>>(m, "BodyNodeDnD")
      .def(
          ::py::init<dart::gui::Viewer*, dart::dynamics::BodyNode*>(),
          ::py::arg("viewer"),
          ::py::arg("bn"))
      .def(
          ::py::init<dart::gui::Viewer*, dart::dynamics::BodyNode*, bool>(),
          ::py::arg("viewer"),
          ::py::arg("bn"),
          ::py::arg("useExternalIK"))
      .def(
          ::py::
              init<dart::gui::Viewer*, dart::dynamics::BodyNode*, bool, bool>(),
          ::py::arg("viewer"),
          ::py::arg("bn"),
          ::py::arg("useExternalIK"),
          ::py::arg("useWholeBody"))
      .def(
          "update", +[](dart::gui::BodyNodeDnD* self) { self->update(); })
      .def(
          "move", +[](dart::gui::BodyNodeDnD* self) { self->move(); })
      .def(
          "saveState", +[](dart::gui::BodyNodeDnD* self) { self->saveState(); })
      .def(
          "release", +[](dart::gui::BodyNodeDnD* self) { self->release(); })
      .def(
          "useExternalIK",
          +[](dart::gui::BodyNodeDnD* self, bool external) {
            self->useExternalIK(external);
          },
          ::py::arg("external"))
      .def(
          "isUsingExternalIK",
          +[](const dart::gui::BodyNodeDnD* self) -> bool {
            return self->isUsingExternalIK();
          })
      .def(
          "useWholeBody",
          +[](dart::gui::BodyNodeDnD* self, bool wholeBody) {
            self->useWholeBody(wholeBody);
          },
          ::py::arg("wholeBody"))
      .def(
          "isUsingWholeBody",
          +[](const dart::gui::BodyNodeDnD* self) -> bool {
            return self->isUsingWholeBody();
          })
      .def(
          "setPreserveOrientationModKey",
          +[](dart::gui::BodyNodeDnD* self,
              osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self->setPreserveOrientationModKey(modkey);
          },
          ::py::arg("modkey"))
      .def(
          "getPreserveOrientationModKey",
          +[](const dart::gui::BodyNodeDnD* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getPreserveOrientationModKey();
          })
      .def(
          "setJointRestrictionModKey",
          +[](dart::gui::BodyNodeDnD* self,
              osgGA::GUIEventAdapter::ModKeyMask modkey) {
            self->setJointRestrictionModKey(modkey);
          },
          ::py::arg("modkey"))
      .def(
          "getJointRestrictionModKey",
          +[](const dart::gui::BodyNodeDnD* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getJointRestrictionModKey();
          });

  ::py::class_<
      dart::gui::InteractiveFrameDnD,
      dart::gui::DragAndDrop,
      std::shared_ptr<dart::gui::InteractiveFrameDnD>>(m, "InteractiveFrameDnD")
      .def(
          ::py::init<dart::gui::Viewer*, dart::gui::InteractiveFrame*>(),
          ::py::arg("viewer"),
          ::py::arg("frame"))
      .def(
          "update",
          +[](dart::gui::InteractiveFrameDnD* self) { self->update(); })
      .def(
          "move", +[](dart::gui::InteractiveFrameDnD* self) { self->move(); })
      .def(
          "saveState",
          +[](dart::gui::InteractiveFrameDnD* self) { self->saveState(); });
}

}  // namespace python
}  // namespace dart
