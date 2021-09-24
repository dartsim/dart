/*
 * Copyright (c) 2011-2021, The DART development contributors
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

namespace py = pybind11;

namespace dart {
namespace python {

void Solver(py::module& m)
{
  ::py::class_<dart::optimizer::Solver::Properties>(m, "SolverProperties")
      .def(::py::init<>())
      .def(
          ::py::init<std::shared_ptr<dart::optimizer::Problem>>(),
          ::py::arg("problem"))
      .def(
          ::py::init<std::shared_ptr<dart::optimizer::Problem>, double>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"),
          ::py::arg("numMaxIterations"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"),
          ::py::arg("numMaxIterations"),
          ::py::arg("iterationsPerPrint"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"),
          ::py::arg("numMaxIterations"),
          ::py::arg("iterationsPerPrint"),
          ::py::arg("ostream"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"),
          ::py::arg("numMaxIterations"),
          ::py::arg("iterationsPerPrint"),
          ::py::arg("ostream"),
          ::py::arg("printFinalResult"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimizer::Problem>,
              double,
              std::size_t,
              std::size_t,
              std::ostream*,
              bool,
              const std::string&>(),
          ::py::arg("problem"),
          ::py::arg("tolerance"),
          ::py::arg("numMaxIterations"),
          ::py::arg("iterationsPerPrint"),
          ::py::arg("ostream"),
          ::py::arg("printFinalResult"),
          ::py::arg("resultFile"))
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

  class PySolver : public dart::optimizer::Solver
  {
  public:
    // Inherit the constructors
    using Solver::Solver;

    // Trampoline for virtual function
    bool solve() override
    {
      PYBIND11_OVERLOAD_PURE(
          bool,   // Return type
          Solver, // Parent class
          solve,  // Name of function in C++ (must match Python name)
      );
    }

    // Trampoline for virtual function
    std::string getType() const override
    {
      PYBIND11_OVERLOAD_PURE(
          std::string, // Return type
          Solver,      // Parent class
          getType,     // Name of function in C++ (must match Python name)
      );
    }

    // Trampoline for virtual function
    std::shared_ptr<Solver> clone() const override
    {
      PYBIND11_OVERLOAD_PURE(
          std::shared_ptr<Solver>, // Return type
          Solver,                  // Parent class
          clone, // Name of function in C++ (must match Python name)
      );
    }
  };

  ::py::class_<
      dart::optimizer::Solver,
      PySolver,
      std::shared_ptr<dart::optimizer::Solver>>(m, "Solver")
      .def(py::init<>())
      .def(
          py::init<dart::optimizer::Solver::Properties>(),
          ::py::arg("properties"))
      .def(
          py::init<std::shared_ptr<dart::optimizer::Problem>>(),
          ::py::arg("problem"))
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
          ::py::arg("properties"))
      .def(
          "setProblem",
          +[](dart::optimizer::Solver* self,
              std::shared_ptr<dart::optimizer::Problem> _newProblem) {
            self->setProblem(_newProblem);
          },
          ::py::arg("newProblem"))
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
          ::py::arg("newTolerance"))
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
          ::py::arg("newMax"))
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
          ::py::arg("newRatio"))
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
          ::py::arg("os"))
      .def(
          "setPrintFinalResult",
          +[](dart::optimizer::Solver* self, bool _print) {
            self->setPrintFinalResult(_print);
          },
          ::py::arg("print"))
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
          ::py::arg("resultFile"))
      .def(
          "getResultFileName",
          +[](const dart::optimizer::Solver* self) -> const std::string& {
            return self->getResultFileName();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
