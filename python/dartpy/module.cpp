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

#include <dart/config.hpp>

namespace dart {
namespace python {

void eigen_geometry(nb::module_& m);

void dart_common(nb::module_& m);
void dart_math(nb::module_& m);
void dart_optimizer(nb::module_& m);
void dart_dynamics(nb::module_& m);
void dart_collision(nb::module_& m);
void dart_constraint(nb::module_& m);
void dart_simulation(nb::module_& m);
void dart_utils(nb::module_& m);
void dart_gui(nb::module_& m);
void bind_gui_probe(nb::module_& m);
#ifdef DARTPY_HAS_VIEWER
void dart_viewer(nb::module_& m);
#endif
void bind_demos(nb::module_& m);

NB_MODULE(_dartpy, m)
{
  m.doc() = "The private native extension of dartpy";

#ifdef DARTPY_VERSION_INFO
  m.attr("__version__") = DARTPY_VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif

  // dartpy/__init__.py imports this extension while the package initializes,
  // so the package module is already in sys.modules. Binding into it gives
  // every public type its canonical module (dartpy.dynamics.Skeleton has
  // __module__ "dartpy.dynamics") and registers each submodule in sys.modules
  // under its public name; no dartpy._* module leaks into the public surface.
  nb::module_ dartpy = nb::module_::import_("dartpy");
  dartnb::bindConstructionGuard(dartpy);

  eigen_geometry(dartpy);

  dart_common(dartpy);
  dart_math(dartpy);
  dart_optimizer(dartpy);
  dart_dynamics(dartpy);
  dart_collision(dartpy);
  dart_constraint(dartpy);
  dart_simulation(dartpy);
  dart_utils(dartpy);
  dart_gui(dartpy);
  bind_gui_probe(dartpy);
#ifdef DARTPY_HAS_VIEWER
  dart_viewer(dartpy);
#endif
  bind_demos(dartpy);
}

} // namespace python
} // namespace dart
