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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void Problem(py::module& m)
{
  ::py::class_<
      dart::optimizer::Problem,
      std::shared_ptr<dart::optimizer::Problem>>(m, "Problem")
      .def(::py::init<>())
      .def(::py::init<std::size_t>(), ::py::arg("dim"))
      .def(
          "setDimension",
          +[](dart::optimizer::Problem* self, std::size_t _dim) {
            self->setDimension(_dim);
          },
          ::py::arg("dim"))
      .def(
          "getDimension",
          +[](const dart::optimizer::Problem* self) -> std::size_t {
            return self->getDimension();
          })
      .def(
          "setInitialGuess",
          +[](dart::optimizer::Problem* self,
              const Eigen::VectorXd& _initGuess) {
            self->setInitialGuess(_initGuess);
          },
          ::py::arg("initGuess"))
      .def(
          "getInitialGuess",
          +[](dart::optimizer::Problem* self) -> const Eigen::VectorXd& {
            return self->getInitialGuess();
          })
      .def(
          "addSeed",
          +[](dart::optimizer::Problem* self, const Eigen::VectorXd& _seed) {
            self->addSeed(_seed);
          },
          ::py::arg("seed"))
      .def(
          "clearAllSeeds",
          +[](dart::optimizer::Problem* self) { self->clearAllSeeds(); })
      .def(
          "setLowerBounds",
          +[](dart::optimizer::Problem* self, const Eigen::VectorXd& _lb) {
            self->setLowerBounds(_lb);
          },
          ::py::arg("lb"))
      .def(
          "setUpperBounds",
          +[](dart::optimizer::Problem* self, const Eigen::VectorXd& _ub) {
            self->setUpperBounds(_ub);
          },
          ::py::arg("ub"))
      .def(
          "setObjective",
          +[](dart::optimizer::Problem* self,
              dart::optimizer::FunctionPtr _obj) { self->setObjective(_obj); },
          ::py::arg("obj"))
      .def(
          "getObjective",
          +[](const dart::optimizer::Problem* self)
              -> dart::optimizer::FunctionPtr { return self->getObjective(); })
      .def(
          "addEqConstraint",
          +[](dart::optimizer::Problem* self,
              dart::optimizer::FunctionPtr _eqConst) {
            self->addEqConstraint(_eqConst);
          },
          ::py::arg("eqConst"))
      .def(
          "addIneqConstraint",
          +[](dart::optimizer::Problem* self,
              dart::optimizer::FunctionPtr _ineqConst) {
            self->addIneqConstraint(_ineqConst);
          },
          ::py::arg("ineqConst"))
      .def(
          "getNumEqConstraints",
          +[](const dart::optimizer::Problem* self) -> std::size_t {
            return self->getNumEqConstraints();
          })
      .def(
          "getNumIneqConstraints",
          +[](const dart::optimizer::Problem* self) -> std::size_t {
            return self->getNumIneqConstraints();
          })
      .def(
          "getEqConstraint",
          +[](const dart::optimizer::Problem* self,
              std::size_t _idx) -> dart::optimizer::FunctionPtr {
            return self->getEqConstraint(_idx);
          },
          ::py::arg("idx"))
      .def(
          "getIneqConstraint",
          +[](const dart::optimizer::Problem* self,
              std::size_t _idx) -> dart::optimizer::FunctionPtr {
            return self->getIneqConstraint(_idx);
          },
          ::py::arg("idx"))
      .def(
          "removeEqConstraint",
          +[](dart::optimizer::Problem* self,
              dart::optimizer::FunctionPtr _eqConst) {
            self->removeEqConstraint(_eqConst);
          },
          ::py::arg("eqConst"))
      .def(
          "removeIneqConstraint",
          +[](dart::optimizer::Problem* self,
              dart::optimizer::FunctionPtr _ineqConst) {
            self->removeIneqConstraint(_ineqConst);
          },
          ::py::arg("ineqConst"))
      .def(
          "removeAllEqConstraints",
          +[](dart::optimizer::Problem* self) {
            self->removeAllEqConstraints();
          })
      .def(
          "removeAllIneqConstraints",
          +[](dart::optimizer::Problem* self) {
            self->removeAllIneqConstraints();
          })
      .def(
          "setOptimumValue",
          +[](dart::optimizer::Problem* self, double _val) {
            self->setOptimumValue(_val);
          },
          ::py::arg("val"))
      .def(
          "getOptimumValue",
          +[](const dart::optimizer::Problem* self) -> double {
            return self->getOptimumValue();
          })
      .def(
          "setOptimalSolution",
          +[](dart::optimizer::Problem* self,
              const Eigen::VectorXd& _optParam) {
            self->setOptimalSolution(_optParam);
          },
          ::py::arg("optParam"))
      .def(
          "getOptimalSolution",
          +[](dart::optimizer::Problem* self) -> const Eigen::VectorXd& {
            return self->getOptimalSolution();
          });
}

} // namespace python
} // namespace dart
