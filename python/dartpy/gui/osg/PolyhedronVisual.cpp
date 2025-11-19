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

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

void PolyhedronVisual(py::module& m)
{
  ::py::class_<
      dart::gui::osg::PolyhedronVisual,
      dart::gui::osg::ViewerAttachment,
      ::osg::ref_ptr<dart::gui::osg::PolyhedronVisual>>(m, "PolyhedronVisual")
      .def(py::init<>())
      .def(
          "setVertices",
          +[](dart::gui::osg::PolyhedronVisual* self,
              const std::vector<Eigen::Vector3d>& vertices) {
            self->setVertices(vertices);
          })
      .def(
          "setVerticesMatrix",
          +[](dart::gui::osg::PolyhedronVisual* self,
              const Eigen::Ref<const Eigen::MatrixXd>& vertices) {
            self->setVertices(vertices);
          },
          ::py::arg("vertices"))
      .def(
          "getVertices",
          +[](dart::gui::osg::PolyhedronVisual* self) {
            return self->getVertices();
          })
      .def("clear", &dart::gui::osg::PolyhedronVisual::clear)
      .def(
          "setSurfaceColor", &dart::gui::osg::PolyhedronVisual::setSurfaceColor)
      .def(
          "getSurfaceColor", &dart::gui::osg::PolyhedronVisual::getSurfaceColor)
      .def(
          "setWireframeColor",
          &dart::gui::osg::PolyhedronVisual::setWireframeColor)
      .def(
          "getWireframeColor",
          &dart::gui::osg::PolyhedronVisual::getWireframeColor)
      .def(
          "setWireframeWidth",
          &dart::gui::osg::PolyhedronVisual::setWireframeWidth)
      .def(
          "getWireframeWidth",
          &dart::gui::osg::PolyhedronVisual::getWireframeWidth)
      .def("display", &dart::gui::osg::PolyhedronVisual::display)
      .def("isDisplayed", &dart::gui::osg::PolyhedronVisual::isDisplayed)
      .def("displaySurface", &dart::gui::osg::PolyhedronVisual::displaySurface)
      .def(
          "isSurfaceDisplayed",
          &dart::gui::osg::PolyhedronVisual::isSurfaceDisplayed)
      .def(
          "displayWireframe",
          &dart::gui::osg::PolyhedronVisual::displayWireframe)
      .def(
          "isWireframeDisplayed",
          &dart::gui::osg::PolyhedronVisual::isWireframeDisplayed);
}

} // namespace python
} // namespace dart
