#include "detail/dart_nb.hpp"
#include "detail/eigen.hpp"
#include "gui/osg/ownership.hpp"

#include <nanobind/stl/vector.h>

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

#include <dart/gui/osg/InteractiveFrame.hpp>

#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace dart {
namespace python {

void InteractiveFrame(nb::module_& m)
{
  auto it
      = dartnb::dart_class<
            dart::gui::osg::InteractiveTool,
            dart::dynamics::SimpleFrame>(m, "InteractiveTool")
            .def(
                dartnb::init<
                    dart::gui::osg::InteractiveFrame*,
                    double,
                    const std::string&>(),
                nb::arg("frame").none(),
                nb::arg("defaultAlpha"),
                nb::arg("name"))
            .def(
                "setEnabled",
                +[](dart::gui::osg::InteractiveTool* self, bool enabled) {
                  self->setEnabled(enabled);
                },
                nb::arg("enabled"))
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
                nb::arg("alpha"))
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
                nb::arg("alpha"))
            .def(
                "setDefaultAlpha",
                +[](dart::gui::osg::InteractiveTool* self,
                    double alpha,
                    bool reset) { self->setDefaultAlpha(alpha, reset); },
                nb::arg("alpha"),
                nb::arg("reset"))
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
                },
                nb::rv_policy::reference_internal)
            .def(
                "getShapeFrames",
                +[](const dart::gui::osg::InteractiveTool* self)
                    -> const std::vector<const dart::dynamics::SimpleFrame*> {
                  return self->getShapeFrames();
                },
                nb::rv_policy::reference_internal)
            .def(
                "removeAllShapeFrames",
                +[](dart::gui::osg::InteractiveTool* self) {
                  self->removeAllShapeFrames();
                });

  nb::enum_<dart::gui::osg::InteractiveTool::Type>(
      it, "Type", nb::is_arithmetic())
      .value("LINEAR", dart::gui::osg::InteractiveTool::Type::LINEAR)
      .value("ANGULAR", dart::gui::osg::InteractiveTool::Type::ANGULAR)
      .value("PLANAR", dart::gui::osg::InteractiveTool::Type::PLANAR)
      .value("NUM_TYPES", dart::gui::osg::InteractiveTool::Type::NUM_TYPES)
      .export_values();

  dartnb::
      dart_class<dart::gui::osg::InteractiveFrame, dart::dynamics::SimpleFrame>(
          m, "InteractiveFrame")
          .def(
              dartnb::init<dart::dynamics::Frame*>(),
              nb::arg("referenceFrame").none())
          .def(
              dartnb::init<dart::dynamics::Frame*, const std::string&>(),
              nb::arg("referenceFrame").none(),
              nb::arg("name"))
          .def(
              dartnb::init<
                  dart::dynamics::Frame*,
                  const std::string&,
                  const Eigen::Isometry3d&>(),
              nb::arg("referenceFrame").none(),
              nb::arg("name"),
              nb::arg("relativeTransform"))
          .def(
              dartnb::init<
                  dart::dynamics::Frame*,
                  const std::string&,
                  const Eigen::Isometry3d&,
                  double>(),
              nb::arg("referenceFrame").none(),
              nb::arg("name"),
              nb::arg("relativeTransform"),
              nb::arg("sizeScale"))
          .def(
              dartnb::init<
                  dart::dynamics::Frame*,
                  const std::string&,
                  const Eigen::Isometry3d&,
                  double,
                  double>(),
              nb::arg("referenceFrame").none(),
              nb::arg("name"),
              nb::arg("relativeTransform"),
              nb::arg("sizeScale"),
              nb::arg("thicknessScale"))
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
              nb::arg("sizeScale"))
          .def(
              "resizeStandardVisuals",
              +[](dart::gui::osg::InteractiveFrame* self,
                  double size_scale,
                  double thickness_scale) {
                self->resizeStandardVisuals(size_scale, thickness_scale);
              },
              nb::arg("sizeScale"),
              nb::arg("thicknessScale"))
          .def(
              "getShapeFrames",
              +[](dart::gui::osg::InteractiveFrame* self)
                  -> const std::vector<dart::dynamics::SimpleFrame*> {
                return self->getShapeFrames();
              },
              nb::rv_policy::reference_internal)
          .def(
              "getShapeFrames",
              +[](const dart::gui::osg::InteractiveFrame* self)
                  -> const std::vector<const dart::dynamics::SimpleFrame*> {
                return self->getShapeFrames();
              },
              nb::rv_policy::reference_internal)
          .def(
              "removeAllShapeFrames",
              +[](dart::gui::osg::InteractiveFrame* self) {
                self->removeAllShapeFrames();
              });
}

} // namespace python
} // namespace dart
