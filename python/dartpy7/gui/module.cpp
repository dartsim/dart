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

#include <dart7/gui/renderer.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <gui/module.hpp>

namespace nb = nanobind;

namespace dart7::python {

void defGui(nb::module_& module)
{
  auto gui = module.def_submodule("gui", "Experimental GUI bindings for dart7.");

  gui.attr("is_supported") = nb::bool_(true);

  nb::enum_<dart7::gui::BackingMode>(gui, "BackingMode")
    .value("Headless", dart7::gui::BackingMode::Headless)
    .value("Window", dart7::gui::BackingMode::Window)
    .export_values();

  nb::class_<dart7::gui::RendererOptions>(gui, "RendererOptions")
    .def(nb::init<>())
    .def_rw("mode", &dart7::gui::RendererOptions::mode)
    .def_rw("window_title", &dart7::gui::RendererOptions::windowTitle)
    .def_rw("width", &dart7::gui::RendererOptions::width)
    .def_rw("height", &dart7::gui::RendererOptions::height);

  nb::class_<dart7::gui::Renderer>(gui, "Renderer")
    .def(nb::init<>())
    .def(nb::init<dart7::gui::RendererOptions>(), nb::arg("options"))
    .def("is_headless", &dart7::gui::Renderer::isHeadless)
    .def("render_frame", &dart7::gui::Renderer::renderFrame)
    .def("poll_events", &dart7::gui::Renderer::pollEvents)
    .def_prop_ro(
      "options",
      [](const dart7::gui::Renderer& self) -> const dart7::gui::RendererOptions& {
        return self.options();
      },
      nb::rv_policy::reference_internal
    );
}

} // namespace dart7::python
