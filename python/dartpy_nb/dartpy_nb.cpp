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

#include <nanobind/nanobind.h>

#include "dart/config.hpp"
#include "common/module.hpp"
#include "dynamics/module.hpp"
#include "math/module.hpp"
#include "optimizer/module.hpp"
#include "utils/module.hpp"

namespace nb = nanobind;

NB_MODULE(dartpy_nb, m)
{
  m.doc() = "dartpy_nb: Experimental nanobind bindings for DART";

#ifdef DARTPY_NB_VERSION_INFO
  m.attr("__version__") = DARTPY_NB_VERSION_INFO;
#else
  m.attr("__version__") = DART_VERSION;
#endif

  auto common = m.def_submodule("common", "Common utilities backed by nanobind");
  dart::python_nb::defCommonModule(common);

  auto math = m.def_submodule("math", "Math utilities backed by nanobind");
  dart::python_nb::defMathModule(math);

  auto dynamics = m.def_submodule("dynamics", "Dynamics utilities backed by nanobind");
  dart::python_nb::defDynamicsModule(dynamics);

  auto utils = m.def_submodule("utils", "Utilities backed by nanobind");
  dart::python_nb::defUtilsModule(utils);

  auto optimizer = m.def_submodule("optimizer", "Optimization utilities");
  dart::python_nb::defOptimizerModule(optimizer);
}
