/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>
#include "eigen_pybind.h"

namespace dart {
namespace python {

void Solver(pybind11::module& m)
{
  ::pybind11::class_<dart::optimizer::Solver::Properties>(m, "SolverProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<std::shared_ptr<dart::optimizer::Problem>>(),
          ::pybind11::arg("problem"))
      .def(
          ::pybind11::init<std::shared_ptr<dart::optimizer::Problem>, double>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"))
      .def(
          ::pybind11::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"),
          ::pybind11::arg("numMaxIterations"))
      .def(
          ::pybind11::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"),
          ::pybind11::arg("numMaxIterations"),
          ::pybind11::arg("iterationsPerPrint"))
      .def(
          ::pybind11::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"),
          ::pybind11::arg("numMaxIterations"),
          ::pybind11::arg("iterationsPerPrint"),
          ::pybind11::arg("ostream"))
      .def(
          ::pybind11::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"),
          ::pybind11::arg("numMaxIterations"),
          ::pybind11::arg("iterationsPerPrint"),
          ::pybind11::arg("ostream"),
          ::pybind11::arg("printFinalResult"))
      .def(
          ::pybind11::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool,
              const std::string&>(),
          ::pybind11::arg("problem"),
          ::pybind11::arg("tolerance"),
          ::pybind11::arg("numMaxIterations"),
          ::pybind11::arg("iterationsPerPrint"),
          ::pybind11::arg("ostream"),
          ::pybind11::arg("printFinalResult"),
          ::pybind11::arg("resultFile"))
      .def_readwrite("mProblem", &dart::optimizer::Solver::Properties::mProblem)
      .def_readwrite(
          "mTolerance", &dart::optimizer::Solver::Properties::mTolerance)
      .def_readwrite(
          "mNumMaxIterations",
          &dart::optimizer::Solver::Properties::mNumMaxIterations)
      .def_readwrite(
          "mIterationsPerPrint",
          &dart::optimizer::Solver::Properties::mIterationsPerPrint)
      .def_readwrite(
          "mOutStream", &dart::optimizer::Solver::Properties::mOutStream)
      .def_readwrite(
          "mPrintFinalResult",
          &dart::optimizer::Solver::Properties::mPrintFinalResult)
      .def_readwrite(
          "mResultFile", &dart::optimizer::Solver::Properties::mResultFile);

  ::pybind11::class_<dart::optimizer::Solver>(m, "Solver")
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
          ::pybind11::arg("properties"))
      .def(
          "setProblem",
          +[](dart::optimizer::Solver* self,
              std::shared_ptr<dart::optimizer::Problem> _newProblem) {
            self->setProblem(_newProblem);
          },
          ::pybind11::arg("newProblem"))
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
          ::pybind11::arg("newTolerance"))
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
          ::pybind11::arg("newMax"))
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
          ::pybind11::arg("newRatio"))
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
          ::pybind11::arg("os"))
      .def(
          "setPrintFinalResult",
          +[](dart::optimizer::Solver* self, bool _print) {
            self->setPrintFinalResult(_print);
          },
          ::pybind11::arg("print"))
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
          ::pybind11::arg("resultFile"))
      .def(
          "getResultFileName",
          +[](const dart::optimizer::Solver* self) -> const std::string& {
            return self->getResultFileName();
          },
          ::pybind11::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
