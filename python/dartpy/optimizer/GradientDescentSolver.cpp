/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void GradientDescentSolver(py::module& m)
{
  ::py::class_<dart::optimizer::GradientDescentSolver::UniqueProperties>(
      m, "GradientDescentSolverUniqueProperties")
      .def(::py::init<>())
      .def(::py::init<double>(), ::py::arg("stepMultiplier"))
      .def(
          ::py::init<double, std::size_t>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"))
      .def(
          ::py::init<double, std::size_t, std::size_t>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"))
      .def(
          ::py::init<double, std::size_t, std::size_t, double>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"),
          ::py::arg("maxPerturbationFactor"))
      .def(
          ::py::init<double, std::size_t, std::size_t, double, double>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"),
          ::py::arg("maxPerturbationFactor"),
          ::py::arg("maxRandomizationStep"))
      .def(
          ::py::
              init<double, std::size_t, std::size_t, double, double, double>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"),
          ::py::arg("maxPerturbationFactor"),
          ::py::arg("maxRandomizationStep"),
          ::py::arg("defaultConstraintWeight"))
      .def(
          ::py::init<
              double,
              std::size_t,
              std::size_t,
              double,
              double,
              double,
              Eigen::VectorXd>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"),
          ::py::arg("maxPerturbationFactor"),
          ::py::arg("maxRandomizationStep"),
          ::py::arg("defaultConstraintWeight"),
          ::py::arg("eqConstraintWeights"))
      .def(
          ::py::init<
              double,
              std::size_t,
              std::size_t,
              double,
              double,
              double,
              Eigen::VectorXd,
              Eigen::VectorXd>(),
          ::py::arg("stepMultiplier"),
          ::py::arg("maxAttempts"),
          ::py::arg("perturbationStep"),
          ::py::arg("maxPerturbationFactor"),
          ::py::arg("maxRandomizationStep"),
          ::py::arg("defaultConstraintWeight"),
          ::py::arg("eqConstraintWeights"),
          ::py::arg("ineqConstraintWeights"))
      .def_readwrite(
          "mStepSize",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::mStepSize)
      .def_readwrite(
          "mMaxAttempts",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxAttempts)
      .def_readwrite(
          "mPerturbationStep",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mPerturbationStep)
      .def_readwrite(
          "mMaxPerturbationFactor",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxPerturbationFactor)
      .def_readwrite(
          "mMaxRandomizationStep",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxRandomizationStep)
      .def_readwrite(
          "mDefaultConstraintWeight",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mDefaultConstraintWeight)
      .def_readwrite(
          "mEqConstraintWeights",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mEqConstraintWeights)
      .def_readwrite(
          "mIneqConstraintWeights",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mIneqConstraintWeights);

  ::py::class_<
      dart::optimizer::GradientDescentSolver::Properties,
      dart::optimizer::Solver::Properties,
      dart::optimizer::GradientDescentSolver::UniqueProperties>(
      m, "GradientDescentSolverProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::optimizer::Solver::Properties&>(),
          ::py::arg("solverProperties"))
      .def(
          ::py::init<
              const dart::optimizer::Solver::Properties&,
              const dart::optimizer::GradientDescentSolver::
                  UniqueProperties&>(),
          ::py::arg("solverProperties"),
          ::py::arg("descentProperties"));

  ::py::class_<
      dart::optimizer::GradientDescentSolver,
      dart::optimizer::Solver,
      std::shared_ptr<dart::optimizer::GradientDescentSolver>>(
      m, "GradientDescentSolver")
      .def(::py::init<>())
      .def(
          ::py::init<
              const dart::optimizer::GradientDescentSolver::Properties&>(),
          ::py::arg("properties"))
      .def(
          ::py::init<std::shared_ptr<dart::optimizer::Problem>>(),
          ::py::arg("problem"))
      .def(
          "solve",
          +[](dart::optimizer::GradientDescentSolver* self) -> bool {
            return self->solve();
          })
      .def(
          "getLastConfiguration",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> Eigen::VectorXd { return self->getLastConfiguration(); })
      .def(
          "getType",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::string { return self->getType(); })
      .def(
          "clone",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::shared_ptr<dart::optimizer::Solver> {
            return self->clone();
          })
      .def(
          "setProperties",
          +[](dart::optimizer::GradientDescentSolver* self,
              const dart::optimizer::GradientDescentSolver::Properties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::optimizer::GradientDescentSolver* self,
              const dart::optimizer::GradientDescentSolver::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "getGradientDescentProperties",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> dart::optimizer::GradientDescentSolver::Properties {
            return self->getGradientDescentProperties();
          })
      .def(
          "setStepSize",
          +[](dart::optimizer::GradientDescentSolver* self,
              double _newMultiplier) { self->setStepSize(_newMultiplier); },
          ::py::arg("newMultiplier"))
      .def(
          "getStepSize",
          +[](const dart::optimizer::GradientDescentSolver* self) -> double {
            return self->getStepSize();
          })
      .def(
          "setMaxAttempts",
          +[](dart::optimizer::GradientDescentSolver* self,
              std::size_t _maxAttempts) { self->setMaxAttempts(_maxAttempts); },
          ::py::arg("maxAttempts"))
      .def(
          "getMaxAttempts",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getMaxAttempts(); })
      .def(
          "setPerturbationStep",
          +[](dart::optimizer::GradientDescentSolver* self, std::size_t _step) {
            self->setPerturbationStep(_step);
          },
          ::py::arg("step"))
      .def(
          "getPerturbationStep",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getPerturbationStep(); })
      .def(
          "setMaxPerturbationFactor",
          +[](dart::optimizer::GradientDescentSolver* self, double _factor) {
            self->setMaxPerturbationFactor(_factor);
          },
          ::py::arg("factor"))
      .def(
          "getMaxPerturbationFactor",
          +[](const dart::optimizer::GradientDescentSolver* self) -> double {
            return self->getMaxPerturbationFactor();
          })
      .def(
          "setDefaultConstraintWeight",
          +[](dart::optimizer::GradientDescentSolver* self,
              double _newDefault) {
            self->setDefaultConstraintWeight(_newDefault);
          },
          ::py::arg("newDefault"))
      .def(
          "getDefaultConstraintWeight",
          +[](const dart::optimizer::GradientDescentSolver* self) -> double {
            return self->getDefaultConstraintWeight();
          })
      .def(
          "randomizeConfiguration",
          +[](dart::optimizer::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->randomizeConfiguration(_x); },
          ::py::arg("x"))
      .def(
          "clampToBoundary",
          +[](dart::optimizer::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->clampToBoundary(_x); },
          ::py::arg("x"))
      .def(
          "getLastNumIterations",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getLastNumIterations(); })
      .def_readonly_static(
          "Type", &dart::optimizer::GradientDescentSolver::Type);
}

} // namespace python
} // namespace dart
