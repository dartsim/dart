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

#ifndef DART_TESTS_LCP_SOLVER_MANIFEST_HPP_
#define DART_TESTS_LCP_SOLVER_MANIFEST_HPP_

#include <dart/math/lcp/all.hpp>

#include <array>
#include <memory>
#include <string_view>

#include <cstddef>

namespace dart::test {

enum class LcpProblemSupport
{
  Standard,
  Boxed,
  FrictionIndex
};

struct LcpSolverManifestEntry
{
  std::string_view name;
  std::string_view family;
  bool supportsStandard;
  bool supportsBoxed;
  bool supportsFrictionIndex;
  std::unique_ptr<dart::math::LcpSolver> (*create)();
};

template <typename Solver>
std::unique_ptr<dart::math::LcpSolver> createLcpSolver()
{
  return std::make_unique<Solver>();
}

inline constexpr std::array<LcpSolverManifestEntry, 24> kLcpSolverManifest{{
    {"Dantzig",
     "Pivoting",
     true,
     true,
     true,
     &createLcpSolver<dart::math::DantzigSolver>},
    {"Lemke",
     "Pivoting",
     true,
     false,
     false,
     &createLcpSolver<dart::math::LemkeSolver>},
    {"Baraff",
     "Pivoting",
     true,
     false,
     false,
     &createLcpSolver<dart::math::BaraffSolver>},
    {"Direct",
     "Pivoting",
     true,
     false,
     false,
     &createLcpSolver<dart::math::DirectSolver>},
    {"Pgs",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::PgsSolver>},
    {"SymmetricPsor",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::SymmetricPsorSolver>},
    {"Jacobi",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::JacobiSolver>},
    {"RedBlackGaussSeidel",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::RedBlackGaussSeidelSolver>},
    {"BlockedJacobi",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::BlockedJacobiSolver>},
    {"BGS",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::BgsSolver>},
    {"NNCG",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::NncgSolver>},
    {"SubspaceMinimization",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::SubspaceMinimizationSolver>},
    {"Apgd",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::ApgdSolver>},
    {"Tgs",
     "Projection",
     true,
     true,
     true,
     &createLcpSolver<dart::math::TgsSolver>},
    {"MinimumMapNewton",
     "Newton",
     true,
     false,
     false,
     &createLcpSolver<dart::math::MinimumMapNewtonSolver>},
    {"FischerBurmeisterNewton",
     "Newton",
     true,
     false,
     false,
     &createLcpSolver<dart::math::FischerBurmeisterNewtonSolver>},
    {"PenalizedFischerBurmeisterNewton",
     "Newton",
     true,
     false,
     false,
     &createLcpSolver<dart::math::PenalizedFischerBurmeisterNewtonSolver>},
    {"InteriorPoint",
     "Other",
     true,
     false,
     false,
     &createLcpSolver<dart::math::InteriorPointSolver>},
    {"MPRGP",
     "Other",
     true,
     false,
     false,
     &createLcpSolver<dart::math::MprgpSolver>},
    {"ShockPropagation",
     "Other",
     true,
     true,
     true,
     &createLcpSolver<dart::math::ShockPropagationSolver>},
    {"Staggering",
     "Other",
     false,
     false,
     true,
     &createLcpSolver<dart::math::StaggeringSolver>},
    {"Admm",
     "Other",
     true,
     true,
     true,
     &createLcpSolver<dart::math::AdmmSolver>},
    {"Sap", "Other", true, true, true, &createLcpSolver<dart::math::SapSolver>},
    {"BoxedSemiSmoothNewton",
     "Newton",
     true,
     true,
     true,
     &createLcpSolver<dart::math::BoxedSemiSmoothNewtonSolver>},
}};

inline constexpr std::size_t kLcpSolverIdentitySchemaVersion = 1;

inline constexpr std::size_t getLcpSolverManifestIndex(std::string_view name)
{
  for (std::size_t i = 0; i < kLcpSolverManifest.size(); ++i) {
    if (kLcpSolverManifest[i].name == name) {
      return i + 1;
    }
  }
  return 0;
}

inline constexpr bool supportsProblem(
    const LcpSolverManifestEntry& solver, LcpProblemSupport support)
{
  switch (support) {
    case LcpProblemSupport::Standard:
      return solver.supportsStandard;
    case LcpProblemSupport::Boxed:
      return solver.supportsBoxed;
    case LcpProblemSupport::FrictionIndex:
      return solver.supportsFrictionIndex;
  }

  return false;
}

inline constexpr std::size_t countSolversSupporting(LcpProblemSupport support)
{
  std::size_t count = 0;
  for (const auto& solver : kLcpSolverManifest) {
    if (supportsProblem(solver, support)) {
      ++count;
    }
  }
  return count;
}

} // namespace dart::test

#endif // DART_TESTS_LCP_SOLVER_MANIFEST_HPP_
