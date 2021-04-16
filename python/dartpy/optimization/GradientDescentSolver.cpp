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
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void GradientDescentSolver(py::module& m)
{
  ::py::class_<dart::optimization::GradientDescentSolver::UniqueProperties>(
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
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mStepSize)
      .def_readwrite(
          "mMaxAttempts",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mMaxAttempts)
      .def_readwrite(
          "mPerturbationStep",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mPerturbationStep)
      .def_readwrite(
          "mMaxPerturbationFactor",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mMaxPerturbationFactor)
      .def_readwrite(
          "mMaxRandomizationStep",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mMaxRandomizationStep)
      .def_readwrite(
          "mDefaultConstraintWeight",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mDefaultConstraintWeight)
      .def_readwrite(
          "mEqConstraintWeights",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mEqConstraintWeights)
      .def_readwrite(
          "mIneqConstraintWeights",
          &dart::optimization::GradientDescentSolver::UniqueProperties::
              mIneqConstraintWeights);

  ::py::class_<
      dart::optimization::GradientDescentSolver::Properties,
      dart::optimization::Solver::Properties,
      dart::optimization::GradientDescentSolver::UniqueProperties>(
      m, "GradientDescentSolverProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::optimization::Solver::Properties&>(),
          ::py::arg("solverProperties"))
      .def(
          ::py::init<
              const dart::optimization::Solver::Properties&,
              const dart::optimization::GradientDescentSolver::
                  UniqueProperties&>(),
          ::py::arg("solverProperties"),
          ::py::arg("descentProperties"));

  ::py::class_<
      dart::optimization::GradientDescentSolver,
      dart::optimization::Solver,
      std::shared_ptr<dart::optimization::GradientDescentSolver>>(
      m, "GradientDescentSolver")
      .def(::py::init<>())
      .def(
          ::py::init<
              const dart::optimization::GradientDescentSolver::Properties&>(),
          ::py::arg("properties"))
      .def(
          ::py::init<std::shared_ptr<dart::optimization::Problem>>(),
          ::py::arg("problem"))
      .def(
          "solve",
          +[](dart::optimization::GradientDescentSolver* self) -> bool {
            return self->solve();
          })
      .def(
          "getLastConfiguration",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> Eigen::VectorXd { return self->getLastConfiguration(); })
      .def(
          "getType",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> std::string { return self->getType(); })
      .def(
          "clone",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> std::shared_ptr<dart::optimization::Solver> {
            return self->clone();
          })
      .def(
          "setProperties",
          +[](dart::optimization::GradientDescentSolver* self,
              const dart::optimization::GradientDescentSolver::Properties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::optimization::GradientDescentSolver* self,
              const dart::optimization::GradientDescentSolver::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "getGradientDescentProperties",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> dart::optimization::GradientDescentSolver::Properties {
            return self->getGradientDescentProperties();
          })
      .def(
          "setStepSize",
          +[](dart::optimization::GradientDescentSolver* self,
              double _newMultiplier) { self->setStepSize(_newMultiplier); },
          ::py::arg("newMultiplier"))
      .def(
          "getStepSize",
          +[](const dart::optimization::GradientDescentSolver* self) -> double {
            return self->getStepSize();
          })
      .def(
          "setMaxAttempts",
          +[](dart::optimization::GradientDescentSolver* self,
              std::size_t _maxAttempts) { self->setMaxAttempts(_maxAttempts); },
          ::py::arg("maxAttempts"))
      .def(
          "getMaxAttempts",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> std::size_t { return self->getMaxAttempts(); })
      .def(
          "setPerturbationStep",
          +[](dart::optimization::GradientDescentSolver* self,
              std::size_t _step) { self->setPerturbationStep(_step); },
          ::py::arg("step"))
      .def(
          "getPerturbationStep",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> std::size_t { return self->getPerturbationStep(); })
      .def(
          "setMaxPerturbationFactor",
          +[](dart::optimization::GradientDescentSolver* self, double _factor) {
            self->setMaxPerturbationFactor(_factor);
          },
          ::py::arg("factor"))
      .def(
          "getMaxPerturbationFactor",
          +[](const dart::optimization::GradientDescentSolver* self) -> double {
            return self->getMaxPerturbationFactor();
          })
      .def(
          "setDefaultConstraintWeight",
          +[](dart::optimization::GradientDescentSolver* self,
              double _newDefault) {
            self->setDefaultConstraintWeight(_newDefault);
          },
          ::py::arg("newDefault"))
      .def(
          "getDefaultConstraintWeight",
          +[](const dart::optimization::GradientDescentSolver* self) -> double {
            return self->getDefaultConstraintWeight();
          })
      .def(
          "randomizeConfiguration",
          +[](dart::optimization::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->randomizeConfiguration(_x); },
          ::py::arg("x"))
      .def(
          "clampToBoundary",
          +[](dart::optimization::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->clampToBoundary(_x); },
          ::py::arg("x"))
      .def(
          "getLastNumIterations",
          +[](const dart::optimization::GradientDescentSolver* self)
              -> std::size_t { return self->getLastNumIterations(); })
      .def_readonly_static(
          "Type", &dart::optimization::GradientDescentSolver::Type);
}

} // namespace python
} // namespace dart
