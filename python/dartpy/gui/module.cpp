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

#include "detail/errors.hpp"

#include <dart/gui/RenderingUnavailableError.hpp>

namespace dart {
namespace python {

void dart_gui_filament(nb::module_& m);

void dart_gui(nb::module_& m)
{
  auto sm = m.def_submodule("gui");
  nb::exception<gui::RenderingUnavailableError>(
      sm, "RenderingUnavailableError", dartnb::operationError());
  sm.doc()
      = "Interim bindings of the Filament dart::gui viewer; DART 7's Python "
        "rendering API replaces them before release.";
  dart_gui_filament(sm);
  sm.attr("__dict__")["__builtins__"]
      = nb::module_::import_("builtins").attr("__dict__");
  // Preserve typed out-of-range values for native mouse-index validation.
  nb::exec(
      R"(
import operator as _operator

def _mouse_missing(cls, value):
    try:
        value = _operator.index(value)
    except TypeError:
        return None
    result = int.__new__(cls, value)
    result._name_ = '???'
    result._value_ = value
    return result

for _mouse_enum in [MouseButton, MouseButtonEvent]:
    _mouse_enum._missing_ = classmethod(_mouse_missing)
)",
      sm.attr("__dict__"));
}

} // namespace python
} // namespace dart
