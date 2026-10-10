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

#include <dart/constraint/FbfFrictionSolver.hpp>

namespace dart {
namespace python {

void FbfFrictionSolver(nb::module_& m)
{
  using Solver = constraint::FbfFrictionSolver;

  auto solver = dartnb::dart_class<Solver, constraint::BoxedLcpSolver>(
      m, "FbfFrictionSolver");

  dartnb::dart_class<Solver::Options>(solver, "Options")
      .def(dartnb::init<>())
      .def_rw("boxForAnisotropic", &Solver::Options::boxForAnisotropic)
      .def_rw("maxOuterIterations", &Solver::Options::maxOuterIterations)
      .def_rw("tolerance", &Solver::Options::tolerance)
      .def_rw("stepScale", &Solver::Options::stepScale)
      .def_rw("maxInnerSweeps", &Solver::Options::maxInnerSweeps)
      .def_rw("innerToleranceFactor", &Solver::Options::innerToleranceFactor);

  solver.def(dartnb::init<>())
      .def(dartnb::init<const Solver::Options&>(), nb::arg("options"))
      .def_static("getStaticType", &Solver::getStaticType)
      .def("setOptions", &Solver::setOptions, nb::arg("options"))
      .def("getOptions", &Solver::getOptions, nb::rv_policy::copy)
      .def("reserve", &Solver::reserve, nb::arg("numRows"))
      .def("getStats", &Solver::getStats)
      .def("resetStats", &Solver::resetStats);
}

} // namespace python
} // namespace dart
