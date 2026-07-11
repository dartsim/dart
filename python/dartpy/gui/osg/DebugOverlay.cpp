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

#include <dart/gui/osg/osg.hpp>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

void DebugOverlay(py::module& m)
{
  ::py::class_<
      dart::gui::osg::DebugOverlay,
      dart::gui::osg::ViewerAttachment,
      ::osg::ref_ptr<dart::gui::osg::DebugOverlay>>(m, "DebugOverlay")
      .def(py::init<>())
      .def(
          "setFont",
          +[](dart::gui::osg::DebugOverlay* self, const std::string& fontPath) {
            self->setFont(fontPath);
          },
          ::py::arg("fontPath"))
      .def(
          "getFont",
          +[](const dart::gui::osg::DebugOverlay* self) -> std::string {
            return self->getFont();
          })
      .def(
          "setLineWidth",
          +[](dart::gui::osg::DebugOverlay* self, float width) {
            self->setLineWidth(width);
          },
          ::py::arg("width"))
      .def(
          "getLineWidth",
          +[](const dart::gui::osg::DebugOverlay* self) -> float {
            return self->getLineWidth();
          })
      .def(
          "setCharacterSize",
          +[](dart::gui::osg::DebugOverlay* self, double size) {
            self->setCharacterSize(size);
          },
          ::py::arg("size"))
      .def(
          "getCharacterSize",
          +[](const dart::gui::osg::DebugOverlay* self) -> double {
            return self->getCharacterSize();
          })
      .def(
          "addLine",
          +[](dart::gui::osg::DebugOverlay* self,
              const Eigen::Vector3d& start,
              const Eigen::Vector3d& end,
              const Eigen::Vector4d& color) -> std::size_t {
            return self->addLine(start, end, color);
          },
          ::py::arg("start"),
          ::py::arg("end"),
          ::py::arg("color"))
      .def(
          "addLabel",
          +[](dart::gui::osg::DebugOverlay* self,
              const Eigen::Vector3d& position,
              const std::string& text,
              const Eigen::Vector4d& color) -> std::size_t {
            return self->addLabel(position, text, color);
          },
          ::py::arg("position"),
          ::py::arg("text"),
          ::py::arg("color"))
      .def(
          "addLabel",
          +[](dart::gui::osg::DebugOverlay* self,
              const Eigen::Vector3d& position,
              const std::string& text,
              const Eigen::Vector4d& color,
              double characterSize) -> std::size_t {
            return self->addLabel(position, text, color, characterSize);
          },
          ::py::arg("position"),
          ::py::arg("text"),
          ::py::arg("color"),
          ::py::arg("characterSize"))
      .def(
          "clear", +[](dart::gui::osg::DebugOverlay* self) { self->clear(); })
      .def(
          "getNumLines",
          +[](const dart::gui::osg::DebugOverlay* self) -> std::size_t {
            return self->getNumLines();
          })
      .def(
          "getNumLabels",
          +[](const dart::gui::osg::DebugOverlay* self) -> std::size_t {
            return self->getNumLabels();
          })
      .def(
          "refresh",
          +[](dart::gui::osg::DebugOverlay* self) { self->refresh(); });
}

} // namespace python
} // namespace dart
