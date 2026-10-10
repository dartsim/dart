#include "detail/dart_nb.hpp"

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
#include "gui/osg/probe.hpp"

namespace dart::python {
void bind_gui_probe(nb::module_& m)
{
  namespace probe = dart::python::gui_probe;
  auto sm = m.def_submodule("_probe");
  sm.def("accept_action_adapter", [](osgGA::GUIActionAdapter* action) {
    return dynamic_cast<dart::gui::osg::Viewer*>(action) != nullptr;
  });
  sm.def("base_view", [](dart::gui::osg::Viewer* viewer) -> osgViewer::View* {
    return viewer;
  });
  sm.def(
      "base_subject",
      [](dart::gui::osg::Viewer* viewer) -> dart::common::Subject* {
        return viewer;
      });
  sm.def("refresh", &probe::refresh);
  sm.def("refresh_viewer", [](dart::gui::osg::Viewer* viewer) {
    probe::refreshViewer(viewer);
    dartnb::gui::ownership(viewer)->prune();
  });
  sm.def("shadow_ref_count", [](osgShadow::ShadowTechnique* value) {
    return value->referenceCount();
  });
  sm.def(
      "handle",
      &probe::handle,
      nb::arg("handler"),
      nb::arg("viewer"),
      nb::arg("key") = 65);
  sm.def(
      "dispatch_viewer_handlers",
      [](osgViewer::View* viewer, int key) {
        auto result = probe::dispatchViewerHandlers(viewer, key);
        dartnb::gui::ownership(viewer)->prune();
        return result;
      },
      nb::arg("viewer"),
      nb::arg("key") = 65);
  sm.def(
      "remove_handler",
      [](osgViewer::View* viewer, osgGA::GUIEventHandler* handler) {
        viewer->removeEventHandler(handler);
        dartnb::gui::retire(viewer, handler);
      });
}
} // namespace dart::python
