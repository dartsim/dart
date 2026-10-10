// clang-format off
#include "detail/dart_nb.hpp"
#include "detail/optimizer_properties.hpp"
// clang-format on

#include <nanobind/trampoline.h>

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

#include "eigen_pybind.h"

#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/Solver.hpp>

#include <memory>
#include <ostream>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

void Solver(nb::module_& m)
{
  dartnb::dart_class<dart::optimizer::Solver::Properties>(m, "SolverProperties")
      .def(dartnb::init<>())
      .def(
          dartnb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem").none())
      .def(
          dartnb::init<std::shared_ptr<dart::optimizer::Problem>, double>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"))
      .def(
          dartnb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"))
      .def(
          dartnb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"))
      .def(
          dartnb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream").none())
      .def(
          dartnb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream").none(),
          nb::arg("printFinalResult"))
      .def(
          dartnb::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool,
              const std::string&>(),
          nb::arg("problem").none(),
          nb::arg("tolerance"),
          nb::arg("numMaxIterations"),
          nb::arg("iterationsPerPrint"),
          nb::arg("ostream").none(),
          nb::arg("printFinalResult"),
          nb::arg("resultFile"))
      .def_rw(
          "mProblem",
          &dart::optimizer::Solver::Properties::mProblem,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mProblem))
      .def_rw(
          "mTolerance",
          &dart::optimizer::Solver::Properties::mTolerance,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mTolerance))
      .def_rw(
          "mNumMaxIterations",
          &dart::optimizer::Solver::Properties::mNumMaxIterations,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mNumMaxIterations))
      .def_rw(
          "mIterationsPerPrint",
          &dart::optimizer::Solver::Properties::mIterationsPerPrint,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mIterationsPerPrint))
      .def_rw(
          "mOutStream",
          &dart::optimizer::Solver::Properties::mOutStream,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mOutStream))
      .def_rw(
          "mPrintFinalResult",
          &dart::optimizer::Solver::Properties::mPrintFinalResult,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mPrintFinalResult))
      .def_rw(
          "mResultFile",
          &dart::optimizer::Solver::Properties::mResultFile,
          dartnb::setterArgument(
              &dart::optimizer::Solver::Properties::mResultFile));

  class PySolver : public dart::optimizer::Solver
  {
  public:
    // Inherit the constructors
    NB_TRAMPOLINE(Solver);

    // Trampoline for virtual function
    bool solve() override
    {
      NB_OVERRIDE_PURE(solve);
    }

    // Trampoline for virtual function
    std::string getType() const override
    {
      NB_OVERRIDE_PURE(getType);
    }

    // Trampoline for virtual function
    std::shared_ptr<Solver> clone() const override
    {
      NB_OVERRIDE_PURE(clone);
    }
  };

  dartnb::dart_class<dart::optimizer::Solver, PySolver>(m, "Solver")
      .def(dartnb::init<>())
      .def(
          dartnb::init<dart::optimizer::Solver::Properties>(),
          nb::arg("properties"))
      .def(
          dartnb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem").none())
      .def(
          "solve",
          +[](dart::optimizer::Solver* self) -> bool { return self->solve(); })
      .def(
          "getType",
          +[](const dart::optimizer::Solver* self) -> std::string {
            return self->getType();
          })
      .def(
          "clone",
          +[](const dart::optimizer::Solver* self)
              -> std::shared_ptr<dart::optimizer::Solver> {
            return self->clone();
          })
      .def(
          "setProperties",
          +[](dart::optimizer::Solver* self,
              const dart::optimizer::Solver::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "setProblem",
          +[](dart::optimizer::Solver* self,
              std::shared_ptr<dart::optimizer::Problem> _newProblem) {
            self->setProblem(_newProblem);
          },
          nb::arg("newProblem").none())
      .def(
          "getProblem",
          +[](const dart::optimizer::Solver* self)
              -> std::shared_ptr<dart::optimizer::Problem> {
            return self->getProblem();
          })
      .def(
          "setTolerance",
          +[](dart::optimizer::Solver* self, double _newTolerance) {
            self->setTolerance(_newTolerance);
          },
          nb::arg("newTolerance"))
      .def(
          "getTolerance",
          +[](const dart::optimizer::Solver* self) -> double {
            return self->getTolerance();
          })
      .def(
          "setNumMaxIterations",
          +[](dart::optimizer::Solver* self, std::size_t _newMax) {
            self->setNumMaxIterations(_newMax);
          },
          nb::arg("newMax"))
      .def(
          "getNumMaxIterations",
          +[](const dart::optimizer::Solver* self) -> std::size_t {
            return self->getNumMaxIterations();
          })
      .def(
          "setIterationsPerPrint",
          +[](dart::optimizer::Solver* self, std::size_t _newRatio) {
            self->setIterationsPerPrint(_newRatio);
          },
          nb::arg("newRatio"))
      .def(
          "getIterationsPerPrint",
          +[](const dart::optimizer::Solver* self) -> std::size_t {
            return self->getIterationsPerPrint();
          })
      .def(
          "setOutStream",
          +[](dart::optimizer::Solver* self, std::ostream* _os) {
            self->setOutStream(_os);
          },
          nb::arg("os").none())
      .def(
          "setPrintFinalResult",
          +[](dart::optimizer::Solver* self, bool _print) {
            self->setPrintFinalResult(_print);
          },
          nb::arg("print"))
      .def(
          "getPrintFinalResult",
          +[](const dart::optimizer::Solver* self) -> bool {
            return self->getPrintFinalResult();
          })
      .def(
          "setResultFileName",
          +[](dart::optimizer::Solver* self, const std::string& _resultFile) {
            self->setResultFileName(_resultFile);
          },
          nb::arg("resultFile"))
      .def(
          "getResultFileName",
          +[](const dart::optimizer::Solver* self) -> const std::string& {
            return self->getResultFileName();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart
