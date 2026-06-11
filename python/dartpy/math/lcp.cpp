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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// nanobind must include Python.h before DART/Eigen headers on glibc; otherwise
// Python's feature macros are redefined under -Werror.
// clang-format off
#include <nanobind/nanobind.h>
#include "dart/math/lcp/all.hpp"
#include "math/lcp.hpp"
// clang-format on

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

using dart::math::AdmmSolver;
using dart::math::ApgdSolver;
using dart::math::BaraffSolver;
using dart::math::BgsSolver;
using dart::math::BlockedJacobiSolver;
using dart::math::BoxedSemiSmoothNewtonSolver;
using dart::math::DantzigSolver;
using dart::math::DirectSolver;
using dart::math::FischerBurmeisterNewtonSolver;
using dart::math::InteriorPointSolver;
using dart::math::JacobiSolver;
using dart::math::LcpOptions;
using dart::math::LcpProblem;
using dart::math::LcpProblemType;
using dart::math::LcpResult;
using dart::math::LcpSolver;
using dart::math::LcpSolverStatus;
using dart::math::LemkeSolver;
using dart::math::MinimumMapNewtonSolver;
using dart::math::MprgpSolver;
using dart::math::NncgSolver;
using dart::math::PenalizedFischerBurmeisterNewtonSolver;
using dart::math::PgsSolver;
using dart::math::RedBlackGaussSeidelSolver;
using dart::math::SapSolver;
using dart::math::ShockPropagationSolver;
using dart::math::StaggeringSolver;
using dart::math::SubspaceMinimizationSolver;
using dart::math::SymmetricPsorSolver;
using dart::math::TgsSolver;

template <typename Solver>
void bindLcpSolverClass(nb::module_& m, const char* name)
{
  nb::class_<Solver, LcpSolver>(m, name).def(
      nb::new_([]() { return std::make_shared<Solver>(); }));
}

std::pair<LcpResult, Eigen::VectorXd> solveLcp(
    LcpSolver& solver,
    const LcpProblem& problem,
    const std::optional<Eigen::VectorXd>& initialGuess,
    const std::optional<LcpOptions>& options)
{
  Eigen::VectorXd x;
  if (initialGuess.has_value()) {
    if (initialGuess->size() != problem.size()) {
      throw nb::value_error("initial_guess must match the LCP problem size");
    }
    x = *initialGuess;
  } else {
    x = Eigen::VectorXd::Zero(problem.size());
  }

  LcpOptions solverOptions = options.value_or(solver.getDefaultOptions());
  LcpResult result;
  {
    nb::gil_scoped_release release;
    result = solver.solve(problem, x, solverOptions);
  }
  return {result, x};
}

} // namespace

void defLcp(nb::module_& m)
{
  nb::enum_<LcpSolverStatus>(m, "LcpSolverStatus")
      .value("SUCCESS", LcpSolverStatus::Success)
      .value("FAILED", LcpSolverStatus::Failed)
      .value("MAX_ITERATIONS", LcpSolverStatus::MaxIterations)
      .value("NUMERICAL_ERROR", LcpSolverStatus::NumericalError)
      .value("INVALID_PROBLEM", LcpSolverStatus::InvalidProblem)
      .value("DEGENERATE", LcpSolverStatus::Degenerate)
      .value("NOT_SOLVED", LcpSolverStatus::NotSolved);

  m.def(
      "lcp_solver_status_to_string",
      [](LcpSolverStatus status) { return dart::math::toString(status); },
      nb::arg("status"));

  nb::enum_<LcpProblemType>(m, "LcpProblemType")
      .value("INVALID", LcpProblemType::Invalid)
      .value("STANDARD", LcpProblemType::Standard)
      .value("BOXED", LcpProblemType::Boxed)
      .value("FRICTION_INDEX", LcpProblemType::FrictionIndex);

  m.def(
      "lcp_problem_type_to_string",
      [](LcpProblemType type) { return dart::math::toString(type); },
      nb::arg("type"));

  nb::class_<LcpResult>(m, "LcpResult")
      .def(nb::init<>())
      .def(nb::init<LcpSolverStatus>(), nb::arg("status"))
      .def_rw("status", &LcpResult::status)
      .def_rw("iterations", &LcpResult::iterations)
      .def_rw("residual", &LcpResult::residual)
      .def_rw("complementarity", &LcpResult::complementarity)
      .def_rw("validated", &LcpResult::validated)
      .def_rw("message", &LcpResult::message)
      .def("succeeded", &LcpResult::succeeded)
      .def("__repr__", [](const LcpResult& self) {
        std::ostringstream os;
        os << "LcpResult(status=" << dart::math::toString(self.status)
           << ", iterations=" << self.iterations
           << ", residual=" << self.residual
           << ", complementarity=" << self.complementarity << ")";
        return os.str();
      });

  nb::class_<LcpOptions>(m, "LcpOptions")
      .def(nb::init<>())
      .def_rw("max_iterations", &LcpOptions::maxIterations)
      .def_rw("absolute_tolerance", &LcpOptions::absoluteTolerance)
      .def_rw("relative_tolerance", &LcpOptions::relativeTolerance)
      .def_rw(
          "complementarity_tolerance", &LcpOptions::complementarityTolerance)
      .def_rw("validate_solution", &LcpOptions::validateSolution)
      .def_rw("relaxation", &LcpOptions::relaxation)
      .def_rw("warm_start", &LcpOptions::warmStart)
      .def_rw("early_termination", &LcpOptions::earlyTermination)
      .def_static(
          "with_relaxation",
          &LcpOptions::withRelaxation,
          nb::arg("relaxation"),
          nb::arg("max_iterations") = 100)
      .def_static("high_accuracy", &LcpOptions::highAccuracy)
      .def_static("real_time", &LcpOptions::realTime);

  nb::class_<LcpProblem>(m, "LcpProblem")
      .def(
          nb::init<Eigen::MatrixXd, Eigen::VectorXd>(),
          nb::arg("A"),
          nb::arg("b"))
      .def(
          nb::init<
              Eigen::MatrixXd,
              Eigen::VectorXd,
              Eigen::VectorXd,
              Eigen::VectorXd>(),
          nb::arg("A"),
          nb::arg("b"),
          nb::arg("lo"),
          nb::arg("hi"))
      .def(
          nb::init<
              Eigen::MatrixXd,
              Eigen::VectorXd,
              Eigen::VectorXd,
              Eigen::VectorXd,
              Eigen::VectorXi>(),
          nb::arg("A"),
          nb::arg("b"),
          nb::arg("lo"),
          nb::arg("hi"),
          nb::arg("findex"))
      .def_ro("A", &LcpProblem::A)
      .def_ro("b", &LcpProblem::b)
      .def_ro("lo", &LcpProblem::lo)
      .def_ro("hi", &LcpProblem::hi)
      .def_ro("findex", &LcpProblem::findex)
      .def("size", &LcpProblem::size)
      .def("empty", &LcpProblem::empty)
      .def("get_type", &LcpProblem::getType, nb::arg("tolerance") = 0.0)
      .def(
          "is_standard_lcp",
          &LcpProblem::isStandardLcp,
          nb::arg("tolerance") = 0.0)
      .def("is_boxed_lcp", &LcpProblem::isBoxedLcp)
      .def("has_friction_index", &LcpProblem::hasFrictionIndex);

  nb::class_<LcpSolver>(m, "LcpSolver")
      .def(
          "solve",
          &solveLcp,
          nb::arg("problem"),
          nb::arg("initial_guess") = nb::none(),
          nb::arg("options") = nb::none())
      .def("get_name", &LcpSolver::getName)
      .def("get_category", &LcpSolver::getCategory)
      .def("supports_standard_lcp", &LcpSolver::supportsStandardLcp)
      .def("supports_boxed_lcp", &LcpSolver::supportsBoxedLcp)
      .def("supports_friction_index", &LcpSolver::supportsFrictionIndex)
      .def("supports_problem", &LcpSolver::supportsProblem, nb::arg("problem"))
      .def_prop_rw(
          "default_options",
          [](const LcpSolver& self) { return self.getDefaultOptions(); },
          [](LcpSolver& self, const LcpOptions& options) {
            self.setDefaultOptions(options);
          });

  bindLcpSolverClass<DantzigSolver>(m, "DantzigSolver");
  bindLcpSolverClass<LemkeSolver>(m, "LemkeSolver");
  bindLcpSolverClass<BaraffSolver>(m, "BaraffSolver");
  bindLcpSolverClass<DirectSolver>(m, "DirectSolver");

  bindLcpSolverClass<PgsSolver>(m, "PgsSolver");
  bindLcpSolverClass<SymmetricPsorSolver>(m, "SymmetricPsorSolver");
  bindLcpSolverClass<JacobiSolver>(m, "JacobiSolver");
  bindLcpSolverClass<RedBlackGaussSeidelSolver>(m, "RedBlackGaussSeidelSolver");
  bindLcpSolverClass<BlockedJacobiSolver>(m, "BlockedJacobiSolver");
  bindLcpSolverClass<BgsSolver>(m, "BgsSolver");
  bindLcpSolverClass<NncgSolver>(m, "NncgSolver");
  bindLcpSolverClass<SubspaceMinimizationSolver>(
      m, "SubspaceMinimizationSolver");
  bindLcpSolverClass<ApgdSolver>(m, "ApgdSolver");
  bindLcpSolverClass<TgsSolver>(m, "TgsSolver");

  bindLcpSolverClass<MinimumMapNewtonSolver>(m, "MinimumMapNewtonSolver");
  bindLcpSolverClass<FischerBurmeisterNewtonSolver>(
      m, "FischerBurmeisterNewtonSolver");
  bindLcpSolverClass<PenalizedFischerBurmeisterNewtonSolver>(
      m, "PenalizedFischerBurmeisterNewtonSolver");
  bindLcpSolverClass<BoxedSemiSmoothNewtonSolver>(
      m, "BoxedSemiSmoothNewtonSolver");

  bindLcpSolverClass<InteriorPointSolver>(m, "InteriorPointSolver");
  bindLcpSolverClass<MprgpSolver>(m, "MprgpSolver");
  bindLcpSolverClass<ShockPropagationSolver>(m, "ShockPropagationSolver");
  bindLcpSolverClass<StaggeringSolver>(m, "StaggeringSolver");
  bindLcpSolverClass<AdmmSolver>(m, "AdmmSolver");
  bindLcpSolverClass<SapSolver>(m, "SapSolver");
}

} // namespace dart::python_nb
