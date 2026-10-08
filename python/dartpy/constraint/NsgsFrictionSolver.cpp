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

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void NsgsFrictionSolver(py::module& m)
{
  using Solver = constraint::NsgsFrictionSolver;
  using Stats = constraint::FrictionSolveStats;

  ::py::class_<Stats>(m, "FrictionSolveStats")
      .def(::py::init<>())
      .def_readonly("numSolves", &Stats::numSolves)
      .def_readonly("numConverged", &Stats::numConverged)
      .def_readonly("numAcceptedAtCap", &Stats::numAcceptedAtCap)
      .def_readonly("numFailed", &Stats::numFailed)
      .def_readonly("numContacts", &Stats::numContacts)
      .def_readonly("numBoxContacts", &Stats::numBoxContacts)
      .def_readonly("numLocalFallbacks", &Stats::numLocalFallbacks)
      .def_readonly("numIterations", &Stats::numIterations)
      .def_readonly("maxViolation", &Stats::maxViolation);

  auto solver = ::py::
      class_<Solver, constraint::BoxedLcpSolver, std::shared_ptr<Solver>>(
          m, "NsgsFrictionSolver");

  ::py::enum_<Solver::Law>(solver, "Law")
      .value("Coulomb", Solver::Law::Coulomb)
      .value("Associated", Solver::Law::Associated)
      .value("Box", Solver::Law::Box);

  ::py::class_<Solver::Options>(solver, "Options")
      .def(::py::init<>())
      .def_readwrite("law", &Solver::Options::law)
      .def_readwrite("boxForAnisotropic", &Solver::Options::boxForAnisotropic)
      .def_readwrite("maxSweeps", &Solver::Options::maxSweeps)
      .def_readwrite("tolerance", &Solver::Options::tolerance);

  solver.def(::py::init<>())
      .def(::py::init<const Solver::Options&>(), ::py::arg("options"))
      .def_static("getStaticType", &Solver::getStaticType)
      .def("setOptions", &Solver::setOptions, ::py::arg("options"))
      .def("getOptions", &Solver::getOptions, ::py::return_value_policy::copy)
      .def("reserve", &Solver::reserve, ::py::arg("numRows"))
      .def("getStats", &Solver::getStats)
      .def("resetStats", &Solver::resetStats);
}

} // namespace python
} // namespace dart
