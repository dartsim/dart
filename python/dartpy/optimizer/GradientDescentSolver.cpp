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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/All.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void GradientDescentSolver(py::module& m)
{
  ::py::class_<dart::math::GradientDescentSolver::UniqueProperties>(
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
          &dart::math::GradientDescentSolver::UniqueProperties::mStepSize)
      .def_readwrite(
          "mMaxAttempts",
          &dart::math::GradientDescentSolver::UniqueProperties::mMaxAttempts)
      .def_readwrite(
          "mPerturbationStep",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mPerturbationStep)
      .def_readwrite(
          "mMaxPerturbationFactor",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mMaxPerturbationFactor)
      .def_readwrite(
          "mMaxRandomizationStep",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mMaxRandomizationStep)
      .def_readwrite(
          "mDefaultConstraintWeight",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mDefaultConstraintWeight)
      .def_readwrite(
          "mEqConstraintWeights",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mEqConstraintWeights)
      .def_readwrite(
          "mIneqConstraintWeights",
          &dart::math::GradientDescentSolver::UniqueProperties::
              mIneqConstraintWeights);

  ::py::class_<
      dart::math::GradientDescentSolver::Properties,
      dart::math::Solver::Properties,
      dart::math::GradientDescentSolver::UniqueProperties>(
      m, "GradientDescentSolverProperties")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::math::Solver::Properties&>(),
          ::py::arg("solverProperties"))
      .def(
          ::py::init<
              const dart::math::Solver::Properties&,
              const dart::math::GradientDescentSolver::UniqueProperties&>(),
          ::py::arg("solverProperties"),
          ::py::arg("descentProperties"));

  ::py::class_<
      dart::math::GradientDescentSolver,
      dart::math::Solver,
      std::shared_ptr<dart::math::GradientDescentSolver>>(
      m, "GradientDescentSolver")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::math::GradientDescentSolver::Properties&>(),
          ::py::arg("properties"))
      .def(
          ::py::init<std::shared_ptr<dart::math::Problem>>(),
          ::py::arg("problem"))
      .def(
          "solve",
          +[](dart::math::GradientDescentSolver* self) -> bool {
            return self->solve();
          })
      .def(
          "getLastConfiguration",
          +[](const dart::math::GradientDescentSolver* self)
              -> Eigen::VectorXd { return self->getLastConfiguration(); })
      .def(
          "getType",
          +[](const dart::math::GradientDescentSolver* self) -> std::string {
            return self->getType();
          })
      .def(
          "clone",
          +[](const dart::math::GradientDescentSolver* self)
              -> std::shared_ptr<dart::math::Solver> { return self->clone(); })
      .def(
          "setProperties",
          +[](dart::math::GradientDescentSolver* self,
              const dart::math::GradientDescentSolver::Properties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "setProperties",
          +[](dart::math::GradientDescentSolver* self,
              const dart::math::GradientDescentSolver::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          ::py::arg("properties"))
      .def(
          "getGradientDescentProperties",
          +[](const dart::math::GradientDescentSolver* self)
              -> dart::math::GradientDescentSolver::Properties {
            return self->getGradientDescentProperties();
          })
      .def(
          "setStepSize",
          +[](dart::math::GradientDescentSolver* self, double _newMultiplier) {
            self->setStepSize(_newMultiplier);
          },
          ::py::arg("newMultiplier"))
      .def(
          "getStepSize",
          +[](const dart::math::GradientDescentSolver* self) -> double {
            return self->getStepSize();
          })
      .def(
          "setMaxAttempts",
          +[](dart::math::GradientDescentSolver* self,
              std::size_t _maxAttempts) { self->setMaxAttempts(_maxAttempts); },
          ::py::arg("maxAttempts"))
      .def(
          "getMaxAttempts",
          +[](const dart::math::GradientDescentSolver* self) -> std::size_t {
            return self->getMaxAttempts();
          })
      .def(
          "setPerturbationStep",
          +[](dart::math::GradientDescentSolver* self, std::size_t _step) {
            self->setPerturbationStep(_step);
          },
          ::py::arg("step"))
      .def(
          "getPerturbationStep",
          +[](const dart::math::GradientDescentSolver* self) -> std::size_t {
            return self->getPerturbationStep();
          })
      .def(
          "setMaxPerturbationFactor",
          +[](dart::math::GradientDescentSolver* self, double _factor) {
            self->setMaxPerturbationFactor(_factor);
          },
          ::py::arg("factor"))
      .def(
          "getMaxPerturbationFactor",
          +[](const dart::math::GradientDescentSolver* self) -> double {
            return self->getMaxPerturbationFactor();
          })
      .def(
          "setDefaultConstraintWeight",
          +[](dart::math::GradientDescentSolver* self, double _newDefault) {
            self->setDefaultConstraintWeight(_newDefault);
          },
          ::py::arg("newDefault"))
      .def(
          "getDefaultConstraintWeight",
          +[](const dart::math::GradientDescentSolver* self) -> double {
            return self->getDefaultConstraintWeight();
          })
      .def(
          "randomizeConfiguration",
          +[](dart::math::GradientDescentSolver* self, Eigen::VectorXd& _x) {
            self->randomizeConfiguration(_x);
          },
          ::py::arg("x"))
      .def(
          "clampToBoundary",
          +[](dart::math::GradientDescentSolver* self, Eigen::VectorXd& _x) {
            self->clampToBoundary(_x);
          },
          ::py::arg("x"))
      .def(
          "getLastNumIterations",
          +[](const dart::math::GradientDescentSolver* self) -> std::size_t {
            return self->getLastNumIterations();
          })
      .def_readonly_static("Type", &dart::math::GradientDescentSolver::Type);
}

} // namespace python
} // namespace dart
