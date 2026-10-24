// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "gui/probe.hpp"

#include "gui/ownership.hpp"

#include <dart/common/Logging.hpp>
#include <dart/common/Observer.hpp>

namespace dart::python {
void bind_gui_probe(nb::module_& m)
{
  auto sm = m.def_submodule("_probe");
  sm.def("has_custom_log_sink", &dart::common::detail::hasCustomLogSink);
  sm.def(
      "base_subject",
      [](dart::common::Subject* subject) -> dart::common::Subject* {
        return subject;
      });
  sm.def(
      "base_observer",
      [](dart::common::Observer* observer) -> dart::common::Observer* {
        return observer;
      });
  sm.def("refresh", &gui_probe::refresh);
  sm.def("refresh_viewer", [](dart::gui::Viewer* viewer) {
    // A Python callback can remove its own node; retain the native snapshot.
    for (const auto& node : dartnb::gui::nodes(viewer))
      node->refresh();
  });
  sm.def(
      "handle",
      &gui_probe::handle,
      nb::arg("handler"),
      nb::arg("viewer"),
      nb::arg("key") = 65);
  sm.def(
      "dispatch_viewer_handlers",
      [](dart::gui::Viewer* viewer, int key) {
        return viewer->handleEvent(dart::gui::GUIEventAdapter(
            dart::gui::GUIEventAdapter::KEYDOWN, key));
      },
      nb::arg("viewer"),
      nb::arg("key") = 65);
  sm.def(
      "remove_handler",
      [](dart::gui::Viewer* viewer,
         const std::shared_ptr<dart::gui::GUIEventHandler>& handler) {
        viewer->removeEventHandler(handler);
        dartnb::gui::retire(viewer, handler.get());
      });
  sm.def(
      "release_gui_ownership_without_thread_state",
      [](dart::gui::Viewer* viewer) {
        auto retired = std::make_unique<dartnb::gui::Ownership>(nullptr);
        retired->active = std::move(dartnb::gui::ownership(viewer)->active);
        for (const auto& edge : retired->active)
          edge.detach();
        nb::gil_scoped_release release;
        retired.reset();
      });
}
} // namespace dart::python
