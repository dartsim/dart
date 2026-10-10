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

#include "gui/osg/ownership.hpp"

#include <dart/gui/osg/GridVisual.hpp>
#include <dart/gui/osg/Viewer.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace dart {
namespace python {

void GridVisual(nb::module_& m)
{
  dartnb::
      dart_class<dart::gui::osg::GridVisual, dart::gui::osg::ViewerAttachment>(
          m, "GridVisual")
          .def(dartnb::gui::init<>())
          .def(
              "setNumCells",
              +[](dart::gui::osg::GridVisual* self, std::size_t cells) {
                self->setNumCells(cells);
              })
          .def(
              "setMinorLineStepSize",
              +[](dart::gui::osg::GridVisual* self, double size) {
                self->setMinorLineStepSize(size);
              })
          .def(
              "setNumMinorLinesPerMajorLine",
              +[](dart::gui::osg::GridVisual* self, std::size_t size) {
                self->setNumMinorLinesPerMajorLine(size);
              })
          .def(
              "setPlaneType",
              +[](dart::gui::osg::GridVisual* self,
                  dart::gui::osg::GridVisual::PlaneType type) {
                self->setPlaneType(type);
              })
          .def(
              "setOffset",
              +[](dart::gui::osg::GridVisual* self,
                  const Eigen::Vector3d& offset) { self->setOffset(offset); });

  auto attr = m.attr("GridVisual");
  nb::enum_<dart::gui::osg::GridVisual::PlaneType>(
      attr, "PlaneType", nb::is_arithmetic())
      .value("XY", dart::gui::osg::GridVisual::PlaneType::XY)
      .value("YZ", dart::gui::osg::GridVisual::PlaneType::YZ)
      .value("ZX", dart::gui::osg::GridVisual::PlaneType::ZX);
}

} // namespace python
} // namespace dart
