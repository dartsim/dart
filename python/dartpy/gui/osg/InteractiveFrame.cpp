/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dart.hpp>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowTechnique>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void InteractiveFrame(py::module& m) {
  auto it
      = ::py::class_<
            dart::gui::osg::InteractiveTool,
            dart::dynamics::SimpleFrame,
            std::shared_ptr<dart::gui::osg::InteractiveTool>>(
            m, "InteractiveTool")
            .def(
                ::py::init<
                    dart::gui::osg::InteractiveFrame*,
                    double,
                    const std::string&>(),
                ::py::arg("frame"),
                ::py::arg("defaultAlpha"),
                ::py::arg("name"))
            .def(
                "setEnabled",
                +[](dart::gui::osg::InteractiveTool* self, bool enabled) {
                  self->setEnabled(enabled);
                },
                ::py::arg("enabled"))
            .def(
                "getEnabled",
                +[](const dart::gui::osg::InteractiveTool* self) -> bool {
                  return self->getEnabled();
                })
            .def(
                "setAlpha",
                +[](dart::gui::osg::InteractiveTool* self, double alpha) {
                  self->setAlpha(alpha);
                },
                ::py::arg("alpha"))
            .def(
                "resetAlpha",
                +[](dart::gui::osg::InteractiveTool* self) {
                  self->resetAlpha();
                })
            .def(
                "setDefaultAlpha",
                +[](dart::gui::osg::InteractiveTool* self, double alpha) {
                  self->setDefaultAlpha(alpha);
                },
                ::py::arg("alpha"))
            .def(
                "setDefaultAlpha",
                +[](dart::gui::osg::InteractiveTool* self,
                    double alpha,
                    bool reset) {
                  self->setDefaultAlpha(alpha, reset);
                },
                ::py::arg("alpha"),
                ::py::arg("reset"))
            .def(
                "getDefaultAlpha",
                +[](const dart::gui::osg::InteractiveTool* self) -> double {
                  return self->getDefaultAlpha();
                })
            .def(
                "getShapeFrames",
                +[](dart::gui::osg::InteractiveTool* self)
                    -> const std::vector<dart::dynamics::SimpleFrame*> {
                  return self->getShapeFrames();
                })
            .def(
                "getShapeFrames",
                +[](const dart::gui::osg::InteractiveTool* self)
                    -> const std::vector<const dart::dynamics::SimpleFrame*> {
                  return self->getShapeFrames();
                })
            .def(
                "removeAllShapeFrames",
                +[](dart::gui::osg::InteractiveTool* self) {
                  self->removeAllShapeFrames();
                });

  ::py::enum_<dart::gui::osg::InteractiveTool::Type>(it, "Type")
      .value("LINEAR", dart::gui::osg::InteractiveTool::Type::LINEAR)
      .value("ANGULAR", dart::gui::osg::InteractiveTool::Type::ANGULAR)
      .value("PLANAR", dart::gui::osg::InteractiveTool::Type::PLANAR)
      .value("NUM_TYPES", dart::gui::osg::InteractiveTool::Type::NUM_TYPES)
      .export_values();

  ::py::class_<
      dart::gui::osg::InteractiveFrame,
      dart::dynamics::SimpleFrame,
      std::shared_ptr<dart::gui::osg::InteractiveFrame>>(m, "InteractiveFrame")
      .def(::py::init<dart::dynamics::Frame*>(), ::py::arg("referenceFrame"))
      .def(
          ::py::init<dart::dynamics::Frame*, const std::string&>(),
          ::py::arg("referenceFrame"),
          ::py::arg("name"))
      .def(
          ::py::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          ::py::arg("referenceFrame"),
          ::py::arg("name"),
          ::py::arg("relativeTransform"))
      .def(
          ::py::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double>(),
          ::py::arg("referenceFrame"),
          ::py::arg("name"),
          ::py::arg("relativeTransform"),
          ::py::arg("sizeScale"))
      .def(
          ::py::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double,
              double>(),
          ::py::arg("referenceFrame"),
          ::py::arg("name"),
          ::py::arg("relativeTransform"),
          ::py::arg("sizeScale"),
          ::py::arg("thicknessScale"))
      .def(
          "resizeStandardVisuals",
          +[](dart::gui::osg::InteractiveFrame* self) {
            self->resizeStandardVisuals();
          })
      .def(
          "resizeStandardVisuals",
          +[](dart::gui::osg::InteractiveFrame* self, double size_scale) {
            self->resizeStandardVisuals(size_scale);
          },
          ::py::arg("sizeScale"))
      .def(
          "resizeStandardVisuals",
          +[](dart::gui::osg::InteractiveFrame* self,
              double size_scale,
              double thickness_scale) {
            self->resizeStandardVisuals(size_scale, thickness_scale);
          },
          ::py::arg("sizeScale"),
          ::py::arg("thicknessScale"))
      .def(
          "getShapeFrames",
          +[](dart::gui::osg::InteractiveFrame* self)
              -> const std::vector<dart::dynamics::SimpleFrame*> {
            return self->getShapeFrames();
          })
      .def(
          "getShapeFrames",
          +[](const dart::gui::osg::InteractiveFrame* self)
              -> const std::vector<const dart::dynamics::SimpleFrame*> {
            return self->getShapeFrames();
          })
      .def(
          "removeAllShapeFrames", +[](dart::gui::osg::InteractiveFrame* self) {
            self->removeAllShapeFrames();
          });
}

} // namespace python
} // namespace dart
