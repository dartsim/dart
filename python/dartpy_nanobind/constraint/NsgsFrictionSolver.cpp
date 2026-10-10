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

#include <dart/constraint/NsgsFrictionSolver.hpp>

namespace dart {
namespace python {

void NsgsFrictionSolver(nb::module_& m)
{
  using Solver = constraint::NsgsFrictionSolver;
  using Stats = constraint::FrictionSolveStats;

  dartnb::dart_class<Stats>(m, "FrictionSolveStats")
      .def(dartnb::init<>())
      .def_ro("numSolves", &Stats::numSolves)
      .def_ro("numConverged", &Stats::numConverged)
      .def_ro("numAcceptedAtCap", &Stats::numAcceptedAtCap)
      .def_ro("numFailed", &Stats::numFailed)
      .def_ro("numContacts", &Stats::numContacts)
      .def_ro("numBoxContacts", &Stats::numBoxContacts)
      .def_ro("numLocalFallbacks", &Stats::numLocalFallbacks)
      .def_ro("numIterations", &Stats::numIterations)
      .def_ro("numInnerIterations", &Stats::numInnerIterations)
      .def_ro("numStepShrinks", &Stats::numStepShrinks)
      .def_ro("numInnerCaps", &Stats::numInnerCaps)
      .def_ro("maxViolation", &Stats::maxViolation);

  auto solver = dartnb::dart_class<Solver, constraint::BoxedLcpSolver>(
      m, "NsgsFrictionSolver");

  nb::enum_<Solver::Law>(solver, "Law", nb::is_arithmetic())
      .value("Coulomb", Solver::Law::Coulomb)
      .value("Associated", Solver::Law::Associated)
      .value("Box", Solver::Law::Box);

  dartnb::dart_class<Solver::Options>(solver, "Options")
      .def(dartnb::init<>())
      .def_rw(
          "law",
          &Solver::Options::law,
          dartnb::setterArgument(&Solver::Options::law))
      .def_rw(
          "boxForAnisotropic",
          &Solver::Options::boxForAnisotropic,
          dartnb::setterArgument(&Solver::Options::boxForAnisotropic))
      .def_rw(
          "maxSweeps",
          &Solver::Options::maxSweeps,
          dartnb::setterArgument(&Solver::Options::maxSweeps))
      .def_rw(
          "tolerance",
          &Solver::Options::tolerance,
          dartnb::setterArgument(&Solver::Options::tolerance));

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
