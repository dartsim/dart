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

void DragAndDrop(pybind11::module& m)
{
  ::pybind11::class_<
      dart::gui::osg::DragAndDrop,
      dart::common::Observer,
      dart::common::Subject,
      ::std::shared_ptr<dart::gui::osg::DragAndDrop>>(m, "DragAndDrop")
      .def("update", +[](dart::gui::osg::DragAndDrop* self) { self->update(); })
      .def(
          "setObstructable",
          +[](dart::gui::osg::DragAndDrop* self, bool _obstructable) {
            self->setObstructable(_obstructable);
          },
          ::pybind11::arg("obstructable"))
      .def(
          "isObstructable",
          +[](const dart::gui::osg::DragAndDrop* self) -> bool {
            return self->isObstructable();
          })
      .def("move", +[](dart::gui::osg::DragAndDrop* self) { self->move(); })
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
          ::pybind11::arg("slope"))
      .def(
          "constrainToPlane",
          +[](dart::gui::osg::DragAndDrop* self,
              const Eigen::Vector3d& normal) {
            self->constrainToPlane(normal);
          },
          ::pybind11::arg("normal"))
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
          ::pybind11::arg("option"))
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
          ::pybind11::arg("rotationModKey"))
      .def(
          "getRotationModKey",
          +[](const dart::gui::osg::DragAndDrop* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getRotationModKey();
          });

  auto attr = m.attr("DragAndDrop");

  ::pybind11::enum_<dart::gui::osg::DragAndDrop::RotationOption>(
      attr, "RotationOption")
      .value(
          "HOLD_MODKEY",
          dart::gui::osg::DragAndDrop::RotationOption::HOLD_MODKEY)
      .value(
          "ALWAYS_ON", dart::gui::osg::DragAndDrop::RotationOption::ALWAYS_ON)
      .value(
          "ALWAYS_OFF", dart::gui::osg::DragAndDrop::RotationOption::ALWAYS_OFF)
      .export_values();

  ::pybind11::class_<
      dart::gui::osg::SimpleFrameDnD,
      dart::gui::osg::DragAndDrop,
      ::std::shared_ptr<dart::gui::osg::SimpleFrameDnD>>(m, "SimpleFrameDnD")
      .def(
          ::pybind11::
              init<dart::gui::osg::Viewer*, dart::dynamics::SimpleFrame*>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("frame"))
      .def("move", +[](dart::gui::osg::SimpleFrameDnD* self) { self->move(); })
      .def("saveState", +[](dart::gui::osg::SimpleFrameDnD* self) {
        self->saveState();
      });

  ::pybind11::class_<
      dart::gui::osg::SimpleFrameShapeDnD,
      dart::gui::osg::SimpleFrameDnD,
      std::shared_ptr<dart::gui::osg::SimpleFrameShapeDnD>>(
      m, "SimpleFrameShapeDnD")
      .def(
          ::pybind11::init<
              dart::gui::osg::Viewer*,
              dart::dynamics::SimpleFrame*,
              dart::dynamics::Shape*>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("frame"),
          ::pybind11::arg("shape"))
      .def("update", +[](dart::gui::osg::SimpleFrameShapeDnD* self) {
        self->update();
      });

  ::pybind11::class_<
      dart::gui::osg::BodyNodeDnD,
      dart::gui::osg::DragAndDrop,
      std::shared_ptr<dart::gui::osg::BodyNodeDnD>>(m, "BodyNodeDnD")
      .def(
          ::pybind11::
              init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("bn"))
      .def(
          ::pybind11::
              init<dart::gui::osg::Viewer*, dart::dynamics::BodyNode*, bool>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("bn"),
          ::pybind11::arg("useExternalIK"))
      .def(
          ::pybind11::init<
              dart::gui::osg::Viewer*,
              dart::dynamics::BodyNode*,
              bool,
              bool>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("bn"),
          ::pybind11::arg("useExternalIK"),
          ::pybind11::arg("useWholeBody"))
      .def("update", +[](dart::gui::osg::BodyNodeDnD* self) { self->update(); })
      .def("move", +[](dart::gui::osg::BodyNodeDnD* self) { self->move(); })
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
          ::pybind11::arg("external"))
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
          ::pybind11::arg("wholeBody"))
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
          ::pybind11::arg("modkey"))
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
          ::pybind11::arg("modkey"))
      .def(
          "getJointRestrictionModKey",
          +[](const dart::gui::osg::BodyNodeDnD* self)
              -> osgGA::GUIEventAdapter::ModKeyMask {
            return self->getJointRestrictionModKey();
          });

  ::pybind11::class_<
      dart::gui::osg::InteractiveFrameDnD,
      dart::gui::osg::DragAndDrop,
      std::shared_ptr<dart::gui::osg::InteractiveFrameDnD>>(
      m, "InteractiveFrameDnD")
      .def(
          ::pybind11::init<
              dart::gui::osg::Viewer*,
              dart::gui::osg::InteractiveFrame*>(),
          ::pybind11::arg("viewer"),
          ::pybind11::arg("frame"))
      .def(
          "update",
          +[](dart::gui::osg::InteractiveFrameDnD* self) { self->update(); })
      .def(
          "move",
          +[](dart::gui::osg::InteractiveFrameDnD* self) { self->move(); })
      .def("saveState", +[](dart::gui::osg::InteractiveFrameDnD* self) {
        self->saveState();
      });
}

} // namespace python
} // namespace dart
