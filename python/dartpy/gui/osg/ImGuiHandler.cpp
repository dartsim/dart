/*
 * Copyright (c) 2011-2021, The DART development contributors
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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

void ImGuiHandler(py::module& m)
{
  ::pybind11::class_<
      dart::gui::osg::ImGuiHandler,
      osg::ref_ptr<dart::gui::osg::ImGuiHandler> >(m, "ImGuiHandler")
      .def(::pybind11::init<>())
      .def(
          "newFrame",
          +[](dart::gui::osg::ImGuiHandler* self, osg::RenderInfo& renderInfo) {
            self->newFrame(renderInfo);
          },
          ::pybind11::arg("renderInfo"))
      .def(
          "render",
          +[](dart::gui::osg::ImGuiHandler* self, osg::RenderInfo& renderInfo) {
            self->render(renderInfo);
          },
          ::pybind11::arg("renderInfo"))
      .def(
          "setCameraCallbacks",
          +[](dart::gui::osg::ImGuiHandler* self, osg::Camera* camera) {
            self->setCameraCallbacks(camera);
          },
          ::pybind11::arg("camera"))
      .def(
          "hasWidget",
          +[](const dart::gui::osg::ImGuiHandler* self,
              const std::shared_ptr<dart::gui::osg::ImGuiWidget>& widget)
              -> bool { return self->hasWidget(widget); },
          ::pybind11::arg("widget"))
      .def(
          "addWidget",
          +[](dart::gui::osg::ImGuiHandler* self,
              const std::shared_ptr<dart::gui::osg::ImGuiWidget>& widget) {
            self->addWidget(widget);
          },
          ::pybind11::arg("widget"))
      .def(
          "addWidget",
          +[](dart::gui::osg::ImGuiHandler* self,
              const std::shared_ptr<dart::gui::osg::ImGuiWidget>& widget,
              bool visible) { self->addWidget(widget, visible); },
          ::pybind11::arg("widget"),
          ::pybind11::arg("visible"))
      .def(
          "removeWidget",
          +[](dart::gui::osg::ImGuiHandler* self,
              const std::shared_ptr<dart::gui::osg::ImGuiWidget>& widget) {
            self->removeWidget(widget);
          },
          ::pybind11::arg("widget"))
      .def(
          "removeAllWidget",
          +[](dart::gui::osg::ImGuiHandler* self) { self->removeAllWidget(); })
      .def(
          "handle",
          +[](dart::gui::osg::ImGuiHandler* self,
              const osgGA::GUIEventAdapter& eventAdapter,
              osgGA::GUIActionAdapter& actionAdapter,
              osg::Object* object,
              osg::NodeVisitor* nodeVisitor) -> bool {
            return self->handle(
                eventAdapter, actionAdapter, object, nodeVisitor);
          },
          ::pybind11::arg("eventAdapter"),
          ::pybind11::arg("actionAdapter"),
          ::pybind11::arg("object"),
          ::pybind11::arg("nodeVisitor"));
}

} // namespace python
} // namespace dart
