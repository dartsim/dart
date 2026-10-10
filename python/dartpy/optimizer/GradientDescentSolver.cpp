// clang-format off
#include "detail/dart_nb.hpp"
#include "detail/optimizer_properties.hpp"
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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/optimizer/GradientDescentSolver.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/Solver.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

template <class Cls>
void defGradientDescentSolverUniquePropertyMethods(Cls& cls)
{
  cls.def_rw(
         "mStepSize",
         &dart::optimizer::GradientDescentSolver::UniqueProperties::mStepSize,
         dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                    UniqueProperties::mStepSize))
      .def_rw(
          "mMaxAttempts",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxAttempts,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mMaxAttempts))
      .def_rw(
          "mPerturbationStep",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mPerturbationStep,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mPerturbationStep))
      .def_rw(
          "mMaxPerturbationFactor",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxPerturbationFactor,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mMaxPerturbationFactor))
      .def_rw(
          "mMaxRandomizationStep",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mMaxRandomizationStep,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mMaxRandomizationStep))
      .def_rw(
          "mDefaultConstraintWeight",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mDefaultConstraintWeight,
          dartnb::setterArgument(
              &dart::optimizer::GradientDescentSolver::UniqueProperties::
                  mDefaultConstraintWeight))
      .def_rw(
          "mEqConstraintWeights",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mEqConstraintWeights,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mEqConstraintWeights))
      .def_rw(
          "mIneqConstraintWeights",
          &dart::optimizer::GradientDescentSolver::UniqueProperties::
              mIneqConstraintWeights,
          dartnb::setterArgument(&dart::optimizer::GradientDescentSolver::
                                     UniqueProperties::mIneqConstraintWeights));
}

void GradientDescentSolver(nb::module_& m)
{
  auto uniqueProperties
      = dartnb::dart_class<
            dart::optimizer::GradientDescentSolver::UniqueProperties>(
            m, "GradientDescentSolverUniqueProperties")
            .def(dartnb::init<>())
            .def(dartnb::init<double>(), nb::arg("stepMultiplier"))
            .def(
                dartnb::init<double, std::size_t>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"))
            .def(
                dartnb::init<double, std::size_t, std::size_t>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"))
            .def(
                dartnb::init<double, std::size_t, std::size_t, double>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"),
                nb::arg("maxPerturbationFactor"))
            .def(
                dartnb::
                    init<double, std::size_t, std::size_t, double, double>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"),
                nb::arg("maxPerturbationFactor"),
                nb::arg("maxRandomizationStep"))
            .def(
                dartnb::init<
                    double,
                    std::size_t,
                    std::size_t,
                    double,
                    double,
                    double>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"),
                nb::arg("maxPerturbationFactor"),
                nb::arg("maxRandomizationStep"),
                nb::arg("defaultConstraintWeight"))
            .def(
                dartnb::init<
                    double,
                    std::size_t,
                    std::size_t,
                    double,
                    double,
                    double,
                    Eigen::VectorXd>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"),
                nb::arg("maxPerturbationFactor"),
                nb::arg("maxRandomizationStep"),
                nb::arg("defaultConstraintWeight"),
                nb::arg("eqConstraintWeights"))
            .def(
                dartnb::init<
                    double,
                    std::size_t,
                    std::size_t,
                    double,
                    double,
                    double,
                    Eigen::VectorXd,
                    Eigen::VectorXd>(),
                nb::arg("stepMultiplier"),
                nb::arg("maxAttempts"),
                nb::arg("perturbationStep"),
                nb::arg("maxPerturbationFactor"),
                nb::arg("maxRandomizationStep"),
                nb::arg("defaultConstraintWeight"),
                nb::arg("eqConstraintWeights"),
                nb::arg("ineqConstraintWeights"));
  defGradientDescentSolverUniquePropertyMethods(uniqueProperties);

  auto properties
      = dartnb::dart_class<
            dart::optimizer::GradientDescentSolver::Properties,
            dart::optimizer::Solver::Properties,
            dart::optimizer::GradientDescentSolver::UniqueProperties>(
            m, "GradientDescentSolverProperties")
            .def(dartnb::init<>())
            .def(
                dartnb::init<const dart::optimizer::Solver::Properties&>(),
                nb::arg("solverProperties"))
            .def(
                dartnb::init<
                    const dart::optimizer::Solver::Properties&,
                    const dart::optimizer::GradientDescentSolver::
                        UniqueProperties&>(),
                nb::arg("solverProperties"),
                nb::arg("descentProperties"));
  defGradientDescentSolverUniquePropertyMethods(properties);

  dartnb::dart_class<
      dart::optimizer::GradientDescentSolver,
      dart::optimizer::Solver>(m, "GradientDescentSolver")
      .def(dartnb::init<>())
      .def(
          dartnb::init<
              const dart::optimizer::GradientDescentSolver::Properties&>(),
          nb::arg("properties"))
      .def(
          dartnb::init<std::shared_ptr<dart::optimizer::Problem>>(),
          nb::arg("problem").none())
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
          nb::arg("properties"))
      .def(
          "setProperties",
          +[](dart::optimizer::GradientDescentSolver* self,
              const dart::optimizer::GradientDescentSolver::UniqueProperties&
                  _properties) { self->setProperties(_properties); },
          nb::arg("properties"))
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
          nb::arg("newMultiplier"))
      .def(
          "getStepSize",
          +[](const dart::optimizer::GradientDescentSolver* self) -> double {
            return self->getStepSize();
          })
      .def(
          "setMaxAttempts",
          +[](dart::optimizer::GradientDescentSolver* self,
              std::size_t _maxAttempts) { self->setMaxAttempts(_maxAttempts); },
          nb::arg("maxAttempts"))
      .def(
          "getMaxAttempts",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getMaxAttempts(); })
      .def(
          "setPerturbationStep",
          +[](dart::optimizer::GradientDescentSolver* self, std::size_t _step) {
            self->setPerturbationStep(_step);
          },
          nb::arg("step"))
      .def(
          "getPerturbationStep",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getPerturbationStep(); })
      .def(
          "setMaxPerturbationFactor",
          +[](dart::optimizer::GradientDescentSolver* self, double _factor) {
            self->setMaxPerturbationFactor(_factor);
          },
          nb::arg("factor"))
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
          nb::arg("newDefault"))
      .def(
          "getDefaultConstraintWeight",
          +[](const dart::optimizer::GradientDescentSolver* self) -> double {
            return self->getDefaultConstraintWeight();
          })
      .def(
          "randomizeConfiguration",
          +[](dart::optimizer::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->randomizeConfiguration(_x); },
          nb::arg("x"))
      .def(
          "clampToBoundary",
          +[](dart::optimizer::GradientDescentSolver* self,
              Eigen::VectorXd& _x) { self->clampToBoundary(_x); },
          nb::arg("x"))
      .def(
          "getLastNumIterations",
          +[](const dart::optimizer::GradientDescentSolver* self)
              -> std::size_t { return self->getLastNumIterations(); })
      .def_ro_static("Type", &dart::optimizer::GradientDescentSolver::Type);
}

} // namespace python
} // namespace dart
