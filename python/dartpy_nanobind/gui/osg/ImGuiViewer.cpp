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

#include <dart/gui/osg/ImGuiHandler.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/Utils.hpp>
#include <dart/gui/osg/Viewer.hpp>

#include <Eigen/Core>
#include <osg/Vec4>

namespace dart {
namespace python {

void ImGuiViewer(nb::module_& m)
{
  dartnb::dart_class<dart::gui::osg::ImGuiViewer, dart::gui::osg::Viewer>(
      m, "ImGuiViewer")
      .def(dartnb::gui::init<>())
      .def(
          dartnb::factory([](const Eigen::Vector4d& clearColor) {
            return dartnb::gui::make<::dart::gui::osg::ImGuiViewer>(
                gui::osg::eigToOsgVec4f(clearColor));
          }),
          nb::arg("clearColor"))
      .def(dartnb::gui::init<const osg::Vec4&>(), nb::arg("clearColor"))
      .def(
          "getImGuiHandler",
          +[](dart::gui::osg::ImGuiViewer* self)
              -> ::osg::ref_ptr<dart::gui::osg::ImGuiHandler> {
            return self->getImGuiHandler();
          },
          nb::rv_policy::reference_internal)
      .def(
          "showAbout",
          +[](dart::gui::osg::ImGuiViewer* self) { self->showAbout(); })
      .def(
          "hideAbout",
          +[](dart::gui::osg::ImGuiViewer* self) { self->hideAbout(); });
}

} // namespace python
} // namespace dart
