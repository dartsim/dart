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

#include "collision/module.hpp"
#include "common/module.hpp"
#include "common/type_casters.hpp"
#include "constraint/module.hpp"
#include "dart/config.hpp"
#include "dynamics/module.hpp"

#if defined(DARTPY_HAS_GUI)
  #include "gui/module.hpp"
#endif

#include "math/module.hpp"
#include "optimizer/module.hpp"
#include "simulation/module.hpp"

#if defined(DARTPY_HAS_SIMULATION_EXPERIMENTAL)
  #include "simulation_experimental/module.hpp"
#endif

#include "utils/module.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

NB_MODULE(_dartpy, m)
{
  m.doc() = "dartpy: Nanobind bindings for DART";

#ifdef DARTPY_VERSION_INFO
  m.attr("__version__") = DARTPY_VERSION_INFO;
#else
  m.attr("__version__") = DART_VERSION;
#endif

  auto common
      = m.def_submodule("common", "Common utilities backed by nanobind");
  dart::python_nb::defCommonModule(common);

  auto math = m.def_submodule("math", "Math utilities backed by nanobind");
  dart::python_nb::defMathModule(math);

  auto dynamics
      = m.def_submodule("dynamics", "Dynamics utilities backed by nanobind");
  dart::python_nb::defDynamicsModule(dynamics);

  auto utils = m.def_submodule("utils", "Utilities backed by nanobind");
  dart::python_nb::defUtilsModule(utils);

  auto collision
      = m.def_submodule("collision", "Collision utilities backed by nanobind");
  dart::python_nb::defCollisionModule(collision);

  // The official DART 7 simulation API is the ECS-backed facade, registered
  // directly into `simulation` (dartpy.simulation) and promoted to the flat
  // top-level dartpy.World by _layout.py. The legacy DART 6 World binding is no
  // longer exposed here (it is retained only as the GUI render target below).
  auto simulation = m.def_submodule(
      "simulation", "ECS-backed simulation API (official DART 7 simulation)");
#if defined(DARTPY_HAS_SIMULATION_EXPERIMENTAL)
  dart::python_nb::defSimulationExperimentalModule(simulation);
#endif

  auto constraint = m.def_submodule(
      "constraint", "Constraint utilities backed by nanobind");
  dart::python_nb::defConstraintModule(constraint);

  auto optimizer = m.def_submodule("optimizer", "Optimization utilities");
  dart::python_nb::defOptimizerModule(optimizer);

  // The classic dart::simulation::World is retained ONLY as render plumbing --
  // the ECS simulation World has no direct viewer path, so demos mirror it into
  // a parallel classic World (WorldRenderBridge) that the Filament viewer
  // draws. It is bound as dartpy.gui.RenderWorld (plus its WorldConfig/enums
  // and ConstraintSolver) and is NOT part of the official flat simulation API.
  //
  // This binding is registered unconditionally -- even when DART_BUILD_GUI=OFF
  // -- because the always-built SDF/Skel/MJCF/URDF parsers in dartpy.io still
  // return dart::simulation::WorldPtr, so a headless build needs a registered
  // Python type for those results. Only the viewer itself (defGuiModule) is
  // gated on the GUI build.
  auto gui = m.def_submodule("gui", "GUI utilities backed by nanobind");
  dart::python_nb::defSimulationModule(gui);
#if defined(DARTPY_HAS_GUI)
  dart::python_nb::defGuiModule(gui);
#endif
}
