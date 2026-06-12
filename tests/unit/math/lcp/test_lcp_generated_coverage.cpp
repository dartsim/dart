/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Manifest-driven generated coverage for DART 7 LCP solvers.
 */

#include "tests/common/lcpsolver/lcp_solver_manifest.hpp"
#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <cmath>

namespace {

using dart::math::LcpOptions;
using dart::math::LcpProblem;
using dart::math::LcpSolverStatus;
using dart::test::LcpSolverManifestEntry;

enum class GeneratedFamily
{
  Standard,
  Boxed,
  FrictionIndex
};

enum class ConditioningClass
{
  WellConditioned,
  MildlyIllConditioned,
  NearSingular,
  SingularDegenerate
};

struct GeneratedCase
{
  std::string name;
  LcpProblem problem;
  Eigen::VectorXd expected;
  GeneratedFamily family;
  ConditioningClass conditioning;
  bool coupled{false};
  double couplingScale{1.0};
};

struct SolverRunConfig
{
  explicit SolverRunConfig(
      const std::string_view solverName, const GeneratedCase& testCase)
  {
    options.warmStart = false;
    options.validateSolution = false;
    options.maxIterations = 20000;
    options.absoluteTolerance = 1e-4;
    options.relativeTolerance = 1e-2;
    options.complementarityTolerance
        = (testCase.family == GeneratedFamily::FrictionIndex) ? 2e-2 : 1e-2;
    expectedTolerance
        = (testCase.family == GeneratedFamily::FrictionIndex) ? 3e-2 : 2e-2;
    allowMaxIterations = true;

    if (isExactSolver(solverName)) {
      options.maxIterations = 200;
      options.absoluteTolerance = 1e-8;
      options.relativeTolerance = 1e-6;
      options.complementarityTolerance = 1e-6;
      expectedTolerance = 1e-6;
      allowMaxIterations = false;

      if (testCase.family == GeneratedFamily::FrictionIndex) {
        options.absoluteTolerance = 1e-6;
        options.relativeTolerance = 1e-3;
        options.complementarityTolerance = 1e-3;
        expectedTolerance = 1e-3;
      }
    }

    if (solverName == "Direct") {
      options.absoluteTolerance = 1e-10;
      options.relativeTolerance = 1e-8;
      options.complementarityTolerance = 1e-8;
      expectedTolerance = 1e-8;
    } else if (solverName == "InteriorPoint") {
      options.maxIterations = 200;
      options.absoluteTolerance = 1e-6;
      options.relativeTolerance = 1e-4;
      options.complementarityTolerance = 1e-6;
      expectedTolerance = 1e-4;
      allowMaxIterations = true;
    } else if (solverName == "MPRGP") {
      options.maxIterations = 1000;
      options.absoluteTolerance = 1e-8;
      options.relativeTolerance = 1e-6;
      options.complementarityTolerance = 1e-6;
      expectedTolerance = 1e-5;
      allowMaxIterations = false;
    } else if (solverName == "NNCG") {
      options.maxIterations = 5000;
      nncgParams.pgsIterations = 2;
      nncgParams.restartInterval = 10;
      nncgParams.restartThreshold = 1.0;
      options.customOptions = &nncgParams;
    } else if (solverName == "SubspaceMinimization") {
      options.maxIterations = 5000;
      subspaceParams.pgsIterations = 5;
      options.customOptions = &subspaceParams;
    } else if (solverName == "Admm") {
      options.maxIterations = 5000;
      options.absoluteTolerance = 1e-5;
      options.relativeTolerance = 1e-3;
      options.complementarityTolerance
          = (testCase.family == GeneratedFamily::FrictionIndex) ? 2e-3 : 1e-3;
      expectedTolerance
          = (testCase.family == GeneratedFamily::FrictionIndex) ? 5e-3 : 3e-3;
    } else if (solverName == "Sap") {
      options.maxIterations = 5000;
      options.absoluteTolerance = 1e-5;
      options.relativeTolerance = 1e-3;
      options.complementarityTolerance
          = (testCase.family == GeneratedFamily::FrictionIndex) ? 2e-3 : 1e-3;
      expectedTolerance
          = (testCase.family == GeneratedFamily::FrictionIndex) ? 5e-3 : 3e-3;
      sapParams.regularization = 1e-6;
      sapParams.maxLineSearchIterations = 32;
      options.customOptions = &sapParams;
    } else if (solverName == "BoxedSemiSmoothNewton") {
      options.maxIterations = 100;
      options.absoluteTolerance = 1e-8;
      options.relativeTolerance = 1e-6;
      options.complementarityTolerance = 1e-6;
      expectedTolerance = 1e-6;
      allowMaxIterations = false;
      if (testCase.family == GeneratedFamily::FrictionIndex
          && testCase.coupled) {
        options.maxIterations = 1000;
      }
    } else if (solverName == "ShockPropagation") {
      options.maxIterations = 1;
      options.absoluteTolerance = 1e-8;
      options.relativeTolerance = 1e-6;
      options.complementarityTolerance = 1e-6;
      expectedTolerance = 1e-6;
      allowMaxIterations = false;
      if (testCase.family == GeneratedFamily::FrictionIndex
          && testCase.coupled) {
        options.absoluteTolerance = 1e-6;
        options.relativeTolerance = 1e-3;
        options.complementarityTolerance = 1e-3;
        expectedTolerance = 1e-3;
        allowMaxIterations = true;
      }
      configureShockPropagation(testCase);
      options.customOptions = &shockParams;
    } else if (
        solverName == "MinimumMapNewton"
        || solverName == "FischerBurmeisterNewton"
        || solverName == "PenalizedFischerBurmeisterNewton") {
      options.maxIterations = 100;
    }

    if (testCase.conditioning == ConditioningClass::MildlyIllConditioned) {
      expectedTolerance = std::max(expectedTolerance, 5e-2);
      options.relativeTolerance = std::max(options.relativeTolerance, 5e-3);
      options.complementarityTolerance
          = std::max(options.complementarityTolerance, 5e-3);
      allowMaxIterations = true;
    } else if (
        testCase.conditioning == ConditioningClass::NearSingular
        || testCase.conditioning == ConditioningClass::SingularDegenerate) {
      expectedTolerance = std::max(expectedTolerance, 1e-1);
      options.relativeTolerance = std::max(options.relativeTolerance, 1e-2);
      options.complementarityTolerance
          = std::max(options.complementarityTolerance, 1e-2);
      allowMaxIterations = true;
    }

    if (testCase.conditioning == ConditioningClass::NearSingular
        && testCase.family == GeneratedFamily::Boxed) {
      // The active-bound near-singular boxed packet is intentionally
      // ill-conditioned; residual and complementarity are the contract here.
      expectedTolerance = std::max(expectedTolerance, 1.0);
    }

    if (solverName == "BoxedSemiSmoothNewton") {
      allowMaxIterations = false;
    }
  }

  static bool isExactSolver(const std::string_view solverName)
  {
    return solverName == "Dantzig" || solverName == "Lemke"
           || solverName == "Baraff" || solverName == "Direct"
           || solverName == "MinimumMapNewton"
           || solverName == "FischerBurmeisterNewton"
           || solverName == "PenalizedFischerBurmeisterNewton";
  }

  void configureShockPropagation(const GeneratedCase& testCase)
  {
    const int n = static_cast<int>(testCase.problem.b.size());
    if (testCase.family != GeneratedFamily::FrictionIndex) {
      shockParams.blockSizes = {n};
      shockParams.layers = {{0}};
      return;
    }

    const int numContacts = n / 3;
    if (testCase.coupled) {
      shockParams.blockSizes = {n};
      shockParams.layers = {{0}};
      return;
    }

    shockParams.blockSizes.assign(numContacts, 3);
    shockParams.layers.clear();
    shockParams.layers.reserve(numContacts);
    for (int i = 0; i < numContacts; ++i) {
      shockParams.layers.push_back({i});
    }
  }

  LcpOptions options;
  double expectedTolerance{1e-2};
  bool allowMaxIterations{true};
  dart::math::NncgSolver::Parameters nncgParams;
  dart::math::SubspaceMinimizationSolver::Parameters subspaceParams;
  dart::math::SapSolver::Parameters sapParams;
  dart::math::ShockPropagationSolver::Parameters shockParams;
};

double signedUnitValue(const int i, const int j, const unsigned seed)
{
  const unsigned value = static_cast<unsigned>(
      37 * (i + 1) + 101 * (j + 3) + 17 * static_cast<int>(seed));
  const double scaled = static_cast<double>(value % 19) / 9.0 - 1.0;
  return scaled == 0.0 ? 0.25 : scaled;
}

Eigen::MatrixXd makeDiagonalDominantSpd(
    const int n, const ConditioningClass conditioning, const unsigned seed)
{
  Eigen::VectorXd baseDiag(n);
  double maxScale = 3.0;
  if (conditioning == ConditioningClass::MildlyIllConditioned) {
    maxScale = 250.0;
  } else if (conditioning == ConditioningClass::NearSingular) {
    maxScale = 1e6;
  }
  for (int i = 0; i < n; ++i) {
    const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    if (conditioning == ConditioningClass::NearSingular) {
      baseDiag[i] = std::pow(maxScale, t) / maxScale;
    } else {
      baseDiag[i] = 1.0 + std::pow(maxScale, t);
    }
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const double magnitude
          = 0.0125 * std::min(baseDiag[r], baseDiag[c]) / std::max(1, n - 1);
      const double value = magnitude * signedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }

  for (int r = 0; r < n; ++r) {
    const double offDiagonalSum = A.row(r).cwiseAbs().sum() - std::abs(A(r, r));
    const double diagonalShift
        = (conditioning == ConditioningClass::NearSingular) ? 1e-9 : 0.25;
    A(r, r) = baseDiag[r] + offDiagonalSum + diagonalShift;
  }

  return A;
}

Eigen::VectorXd makeKnownStandardSolution(const int n, const bool activeLower)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    if (activeLower && i % 4 == 0) {
      x[i] = 0.0;
    } else {
      x[i] = 0.1 + 0.04 * static_cast<double>((i % 5) + 1);
    }
  }
  return x;
}

Eigen::VectorXd makeStandardSlack(const int n, const bool activeLower)
{
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    if (activeLower && i % 4 == 0) {
      w[i] = 0.2 + 0.03 * static_cast<double>(i + 1);
    }
  }
  return w;
}

GeneratedCase makeStandardCase(
    const int n,
    const ConditioningClass conditioning,
    const bool activeLower,
    const unsigned seed)
{
  Eigen::MatrixXd A = makeDiagonalDominantSpd(n, conditioning, seed);
  Eigen::VectorXd x = makeKnownStandardSolution(n, activeLower);
  Eigen::VectorXd w = makeStandardSlack(n, activeLower);
  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "standard_n" << n
       << (conditioning == ConditioningClass::MildlyIllConditioned
               ? "_mild"
               : (conditioning == ConditioningClass::NearSingular
                      ? "_near_singular"
                      : "_well"))
       << (activeLower ? "_active_lower" : "_interior");

  const double inf = std::numeric_limits<double>::infinity();
  return GeneratedCase{
      name.str(),
      LcpProblem(
          A,
          b,
          Eigen::VectorXd::Zero(n),
          Eigen::VectorXd::Constant(n, inf),
          Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Standard,
      conditioning};
}

GeneratedCase makeBoxedCase(
    const int n, const ConditioningClass conditioning, const unsigned seed)
{
  Eigen::MatrixXd A = makeDiagonalDominantSpd(n, conditioning, seed);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const int mode = i % 4;
    if (mode == 0) {
      lo[i] = -0.4;
      hi[i] = 0.7;
      x[i] = lo[i];
      w[i] = 0.15 + 0.01 * static_cast<double>(i);
    } else if (mode == 1) {
      lo[i] = -0.5;
      hi[i] = 0.35;
      x[i] = hi[i];
      w[i] = -0.12 - 0.01 * static_cast<double>(i);
    } else if (mode == 2) {
      lo[i] = -0.8;
      hi[i] = 0.8;
      x[i] = -0.1 + 0.05 * static_cast<double>(i % 3);
    } else {
      lo[i] = -0.25;
      hi[i] = inf;
      x[i] = 0.2 + 0.03 * static_cast<double>(i % 5);
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "boxed_n" << n
       << (conditioning == ConditioningClass::MildlyIllConditioned
               ? "_mild"
               : (conditioning == ConditioningClass::NearSingular
                      ? "_near_singular"
                      : "_well"))
       << "_mixed_active_unbounded";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Boxed,
      conditioning};
}

GeneratedCase makeFrictionIndexCase(
    const int numContacts,
    const ConditioningClass conditioning,
    const unsigned seed,
    const bool coupled = false,
    const double couplingScale = 1.0,
    const double normalSlope = 0.15,
    const double nearSingularScale = 1e4)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int contact = 0; contact < numContacts; ++contact) {
    double scale = 2.0;
    if (conditioning == ConditioningClass::MildlyIllConditioned) {
      scale = 25.0;
    } else if (conditioning == ConditioningClass::NearSingular) {
      scale = nearSingularScale;
    }
    const double contactScale
        = 1.0 + 0.1 * static_cast<double>((seed + contact) % 7);
    diag[3 * contact + 0] = contactScale;
    diag[3 * contact + 1] = contactScale * scale;
    diag[3 * contact + 2] = contactScale * scale * scale;
  }

  if (coupled) {
    for (int r = 0; r < n; ++r) {
      for (int c = r + 1; c < n; ++c) {
        const bool sameContact = (r / 3) == (c / 3);
        const double strength = couplingScale * (sameContact ? 0.025 : 0.01);
        const double magnitude
            = strength * std::min(diag[r], diag[c]) / std::max(1, n - 1);
        const double value = magnitude * signedUnitValue(r, c, seed);
        A(r, c) = value;
        A(c, r) = value;
      }
    }
  }

  for (int r = 0; r < n; ++r) {
    const double offDiagonalSum = A.row(r).cwiseAbs().sum();
    A(r, r) = diag[r] + (coupled ? offDiagonalSum : 0.0);
  }

  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int contact = 0; contact < numContacts; ++contact) {
    const int base = 3 * contact;
    const double normal = 0.5 + normalSlope * static_cast<double>(contact + 1);
    const double mu = 0.25 + 0.05 * static_cast<double>((contact % 4) + 1);

    x[base] = normal;
    lo[base] = 0.0;
    hi[base] = inf;

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base;

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base;

    if (contact % 3 == 0) {
      x[base + 1] = 0.25 * mu * normal;
      x[base + 2] = -0.2 * mu * normal;
    } else if (contact % 3 == 1) {
      x[base + 1] = mu * normal;
      x[base + 2] = -0.1 * mu * normal;
      w[base + 1] = -0.2;
    } else {
      x[base + 1] = 0.1 * mu * normal;
      x[base + 2] = -mu * normal;
      w[base + 2] = 0.2;
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "findex_contacts" << numContacts
       << (conditioning == ConditioningClass::MildlyIllConditioned
               ? "_mild"
               : (conditioning == ConditioningClass::NearSingular
                      ? "_near_singular"
                      : "_well"))
       << (coupled ? "_coupled" : "");
  if (coupled && couplingScale != 1.0) {
    name << "_scale" << static_cast<int>(couplingScale);
  }
  if (std::abs(normalSlope - 0.15) > 1e-12) {
    name << "_normal_slope" << static_cast<int>(100.0 * normalSlope);
  }
  if (conditioning == ConditioningClass::NearSingular
      && std::abs(nearSingularScale - 1e4) > 1e-12) {
    name << "_diag_scale" << static_cast<int>(nearSingularScale);
  }
  name << "_mixed_cone_activity";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, findex),
      x,
      GeneratedFamily::FrictionIndex,
      conditioning,
      coupled,
      couplingScale};
}

GeneratedCase makeSingularDegenerateStandardCase(const int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    if (i % 4 == 0) {
      x[i] = 0.0;
      w[i] = 0.25 + 0.02 * static_cast<double>(i + 1);
    } else {
      A(i, i) = 1.0 + 0.15 * static_cast<double>(i % 7);
      x[i] = 0.08 + 0.03 * static_cast<double>((i % 5) + 1);
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "standard_n" << n << "_singular_degenerate_active_lower";

  const double inf = std::numeric_limits<double>::infinity();
  return GeneratedCase{
      name.str(),
      LcpProblem(
          A,
          b,
          Eigen::VectorXd::Zero(n),
          Eigen::VectorXd::Constant(n, inf),
          Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Standard,
      ConditioningClass::SingularDegenerate};
}

GeneratedCase makeSingularDegenerateBoxedCase(const int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int i = 0; i < n; ++i) {
    const int mode = i % 4;
    if (mode == 0) {
      lo[i] = -0.45;
      hi[i] = 0.55;
      x[i] = lo[i];
      w[i] = 0.2 + 0.01 * static_cast<double>(i + 1);
    } else if (mode == 1) {
      lo[i] = -0.35;
      hi[i] = 0.4;
      x[i] = hi[i];
      w[i] = -0.18 - 0.01 * static_cast<double>(i + 1);
    } else if (mode == 2) {
      lo[i] = -0.7;
      hi[i] = 0.75;
      A(i, i) = 1.2 + 0.1 * static_cast<double>(i % 5);
      x[i] = -0.1 + 0.04 * static_cast<double>(i % 4);
    } else {
      lo[i] = -0.25;
      hi[i] = inf;
      A(i, i) = 1.1 + 0.12 * static_cast<double>(i % 6);
      x[i] = 0.15 + 0.03 * static_cast<double>(i % 5);
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "boxed_n" << n << "_singular_degenerate_mixed_active";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Boxed,
      ConditioningClass::SingularDegenerate};
}

GeneratedCase makeSingularDegenerateFrictionIndexCase(const int numContacts)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd normalDiag(numContacts);
  for (int contact = 0; contact < numContacts; ++contact) {
    normalDiag[contact] = 1.0 + 0.1 * static_cast<double>(contact % 5);
  }

  for (int r = 0; r < numContacts; ++r) {
    for (int c = r + 1; c < numContacts; ++c) {
      const double magnitude = 0.02 * std::min(normalDiag[r], normalDiag[c])
                               / std::max(1, numContacts - 1);
      const double value = magnitude * signedUnitValue(r, c, 31006);
      A(3 * r, 3 * c) = value;
      A(3 * c, 3 * r) = value;
    }
  }

  for (int contact = 0; contact < numContacts; ++contact) {
    const int normalRow = 3 * contact;
    A(normalRow, normalRow)
        = normalDiag[contact] + A.row(normalRow).cwiseAbs().sum() + 0.1;
  }

  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int contact = 0; contact < numContacts; ++contact) {
    const int base = 3 * contact;
    const double normal = 0.35 + 0.08 * static_cast<double>(contact + 1);
    const double mu = 0.3 + 0.05 * static_cast<double>(contact % 4);

    x[base] = normal;
    lo[base] = 0.0;
    hi[base] = inf;

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base;
    x[base + 1] = mu * normal;
    w[base + 1] = -0.12 - 0.01 * static_cast<double>(contact);

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base;
    x[base + 2] = -mu * normal;
    w[base + 2] = 0.1 + 0.01 * static_cast<double>(contact);
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "findex_contacts" << numContacts
       << "_singular_degenerate_coupled_tangent_bounds";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, findex),
      x,
      GeneratedFamily::FrictionIndex,
      ConditioningClass::SingularDegenerate,
      true};
}

GeneratedCase makeStandardActiveSetTransitionCase(
    const int n, const unsigned seed)
{
  Eigen::MatrixXd A
      = makeDiagonalDominantSpd(n, ConditioningClass::WellConditioned, seed);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    switch (i % 4) {
      case 0:
        x[i] = 0.0;
        w[i] = 5e-4 * static_cast<double>(i + 1);
        break;
      case 1:
        x[i] = 0.03;
        break;
      case 2:
        x[i] = 0.12 + 0.01 * static_cast<double>(i % 5);
        break;
      case 3:
        x[i] = 0.015 * static_cast<double>((i % 3) + 1);
        break;
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "standard_n" << n << "_active_set_transition";

  const double inf = std::numeric_limits<double>::infinity();
  return GeneratedCase{
      name.str(),
      LcpProblem(
          A,
          b,
          Eigen::VectorXd::Zero(n),
          Eigen::VectorXd::Constant(n, inf),
          Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Standard,
      ConditioningClass::WellConditioned};
}

GeneratedCase makeBoxedActiveSetTransitionCase(const int n, const unsigned seed)
{
  Eigen::MatrixXd A
      = makeDiagonalDominantSpd(n, ConditioningClass::WellConditioned, seed);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    switch (i % 5) {
      case 0:
        lo[i] = -0.3;
        hi[i] = 0.4;
        x[i] = lo[i];
        w[i] = 5e-4 * static_cast<double>(i + 1);
        break;
      case 1:
        lo[i] = -0.25;
        hi[i] = 0.35;
        x[i] = lo[i] + 0.03;
        break;
      case 2:
        lo[i] = -0.45;
        hi[i] = 0.25;
        x[i] = hi[i] - 0.03;
        break;
      case 3:
        lo[i] = -0.5;
        hi[i] = 0.3;
        x[i] = hi[i];
        w[i] = -5e-4 * static_cast<double>(i + 1);
        break;
      case 4:
        lo[i] = -0.6;
        hi[i] = 0.6;
        x[i] = 0.04 * static_cast<double>((i % 3) - 1);
        break;
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "boxed_n" << n << "_active_set_transition";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, Eigen::VectorXi::Constant(n, -1)),
      x,
      GeneratedFamily::Boxed,
      ConditioningClass::WellConditioned};
}

GeneratedCase makeFrictionIndexActiveSetTransitionCase(
    const int numContacts,
    const unsigned seed,
    const double couplingScale = 1.0)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int i = 0; i < n; ++i) {
    diag[i] = 1.0 + 0.2 * static_cast<double>((seed + i) % 5);
  }

  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const bool sameContact = (r / 3) == (c / 3);
      const double strength = couplingScale * (sameContact ? 0.02 : 0.0075);
      const double magnitude
          = strength * std::min(diag[r], diag[c]) / std::max(1, n - 1);
      const double value = magnitude * signedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }
  for (int r = 0; r < n; ++r) {
    A(r, r) = diag[r] + A.row(r).cwiseAbs().sum();
  }

  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int contact = 0; contact < numContacts; ++contact) {
    const int base = 3 * contact;
    const double normal = 0.08 + 0.05 * static_cast<double>(contact + 1);
    const double mu = 0.35 + 0.05 * static_cast<double>(contact % 3);

    x[base] = normal;
    lo[base] = 0.0;
    hi[base] = inf;

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base;

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base;

    if (contact % 4 == 0) {
      x[base + 1] = 0.94 * mu * normal;
      x[base + 2] = -0.03 * mu * normal;
    } else if (contact % 4 == 1) {
      x[base + 1] = -0.92 * mu * normal;
      x[base + 2] = 0.05 * mu * normal;
    } else if (contact % 4 == 2) {
      x[base + 1] = 0.05 * mu * normal;
      x[base + 2] = 0.90 * mu * normal;
    } else {
      x[base + 1] = -0.04 * mu * normal;
      x[base + 2] = -0.93 * mu * normal;
    }
  }

  Eigen::VectorXd b = A * x - w;

  std::ostringstream name;
  name << "findex_contacts" << numContacts << "_coupled_active_set_transition";

  return GeneratedCase{
      name.str(),
      LcpProblem(A, b, lo, hi, findex),
      x,
      GeneratedFamily::FrictionIndex,
      ConditioningClass::WellConditioned,
      true};
}

std::vector<GeneratedCase> makeStandardCases()
{
  return {
      makeStandardCase(1, ConditioningClass::WellConditioned, false, 1101),
      makeStandardCase(2, ConditioningClass::WellConditioned, true, 1102),
      makeStandardCase(4, ConditioningClass::WellConditioned, true, 1104),
      makeStandardCase(8, ConditioningClass::WellConditioned, true, 1108),
      makeStandardCase(16, ConditioningClass::WellConditioned, true, 1116),
      makeStandardCase(8, ConditioningClass::MildlyIllConditioned, true, 2108),
      makeStandardCase(4, ConditioningClass::MildlyIllConditioned, true, 2104)};
}

std::vector<GeneratedCase> makeBoxedCases()
{
  return {
      makeBoxedCase(2, ConditioningClass::WellConditioned, 3202),
      makeBoxedCase(4, ConditioningClass::WellConditioned, 3204),
      makeBoxedCase(8, ConditioningClass::WellConditioned, 3208),
      makeBoxedCase(12, ConditioningClass::WellConditioned, 3212),
      makeBoxedCase(8, ConditioningClass::MildlyIllConditioned, 4208),
      makeBoxedCase(4, ConditioningClass::MildlyIllConditioned, 4204),
      makeBoxedCase(4, ConditioningClass::NearSingular, 5204)};
}

std::vector<GeneratedCase> makeFrictionIndexCases()
{
  return {
      makeFrictionIndexCase(1, ConditioningClass::WellConditioned, 5301),
      makeFrictionIndexCase(2, ConditioningClass::WellConditioned, 5302),
      makeFrictionIndexCase(4, ConditioningClass::WellConditioned, 5304),
      makeFrictionIndexCase(8, ConditioningClass::WellConditioned, 5308)};
}

std::vector<GeneratedCase> makeCoupledFrictionIndexCases()
{
  return {
      makeFrictionIndexCase(2, ConditioningClass::WellConditioned, 6302, true),
      makeFrictionIndexCase(4, ConditioningClass::WellConditioned, 6304, true),
      makeFrictionIndexCase(6, ConditioningClass::WellConditioned, 6306, true),
      makeFrictionIndexCase(
          2, ConditioningClass::MildlyIllConditioned, 7302, true),
      makeFrictionIndexCase(
          4, ConditioningClass::MildlyIllConditioned, 7304, true)};
}

std::vector<GeneratedCase> makeActiveSetTransitionCases()
{
  return {
      makeStandardActiveSetTransitionCase(16, 18016),
      makeBoxedActiveSetTransitionCase(16, 19016),
      makeFrictionIndexActiveSetTransitionCase(6, 20006)};
}

std::vector<GeneratedCase> makeLargerActiveSetTransitionCases()
{
  return {
      makeStandardActiveSetTransitionCase(32, 21032),
      makeBoxedActiveSetTransitionCase(32, 22032),
      makeFrictionIndexActiveSetTransitionCase(8, 23008)};
}

std::vector<GeneratedCase> makeStressActiveSetTransitionCases()
{
  return {
      makeStandardActiveSetTransitionCase(64, 21064),
      makeBoxedActiveSetTransitionCase(64, 22064),
      makeFrictionIndexActiveSetTransitionCase(12, 23012)};
}

std::vector<GeneratedCase> makeExtremeActiveSetTransitionCases()
{
  return {
      makeStandardActiveSetTransitionCase(128, 21128),
      makeBoxedActiveSetTransitionCase(128, 22128),
      makeFrictionIndexActiveSetTransitionCase(16, 23016)};
}

std::vector<GeneratedCase> makeProductionActiveSetTransitionCases()
{
  return {
      makeFrictionIndexActiveSetTransitionCase(24, 23024, 2.0),
      makeFrictionIndexActiveSetTransitionCase(32, 23032, 4.0),
      makeFrictionIndexActiveSetTransitionCase(48, 23048, 8.0),
      makeFrictionIndexActiveSetTransitionCase(64, 23064, 16.0),
      makeFrictionIndexActiveSetTransitionCase(96, 23096, 32.0),
      makeFrictionIndexActiveSetTransitionCase(128, 23128, 32.0),
      makeFrictionIndexActiveSetTransitionCase(192, 23192, 32.0),
      makeFrictionIndexActiveSetTransitionCase(256, 23256, 32.0)};
}

std::vector<GeneratedCase> makeBatchCases()
{
  return {
      makeStandardCase(4, ConditioningClass::WellConditioned, true, 7104),
      makeBoxedCase(4, ConditioningClass::WellConditioned, 7204),
      makeFrictionIndexCase(2, ConditioningClass::WellConditioned, 7302)};
}

std::vector<GeneratedCase> makeNearSingularStandardCases()
{
  return {
      makeStandardCase(2, ConditioningClass::NearSingular, true, 8102),
      makeStandardCase(3, ConditioningClass::NearSingular, true, 8103)};
}

std::vector<GeneratedCase> makeLargerWellConditionedCases()
{
  return {
      makeStandardCase(32, ConditioningClass::WellConditioned, true, 9132),
      makeStandardCase(64, ConditioningClass::WellConditioned, true, 9164),
      makeBoxedCase(32, ConditioningClass::WellConditioned, 9232),
      makeFrictionIndexCase(16, ConditioningClass::WellConditioned, 9316),
      makeFrictionIndexCase(8, ConditioningClass::WellConditioned, 9408, true)};
}

std::vector<GeneratedCase> makeProductionScaleWellConditionedCases()
{
  return {
      makeStandardCase(128, ConditioningClass::WellConditioned, true, 10128),
      makeBoxedCase(64, ConditioningClass::WellConditioned, 11064),
      makeFrictionIndexCase(24, ConditioningClass::WellConditioned, 12024),
      makeFrictionIndexCase(
          12, ConditioningClass::WellConditioned, 13012, true)};
}

std::vector<GeneratedCase> makeLargerMildlyIllConditionedCases()
{
  return {
      makeStandardCase(
          32, ConditioningClass::MildlyIllConditioned, true, 24032),
      makeStandardCase(
          64, ConditioningClass::MildlyIllConditioned, true, 24064),
      makeBoxedCase(16, ConditioningClass::MildlyIllConditioned, 25016),
      makeBoxedCase(32, ConditioningClass::MildlyIllConditioned, 25032),
      makeFrictionIndexCase(8, ConditioningClass::MildlyIllConditioned, 26008),
      makeFrictionIndexCase(
          6, ConditioningClass::MildlyIllConditioned, 27006, true),
      makeFrictionIndexCase(
          8, ConditioningClass::MildlyIllConditioned, 27008, true),
      makeFrictionIndexCase(
          12, ConditioningClass::MildlyIllConditioned, 27012, true),
      makeFrictionIndexCase(
          16, ConditioningClass::MildlyIllConditioned, 27016, true),
      makeFrictionIndexCase(
          24, ConditioningClass::MildlyIllConditioned, 27024, true),
      makeFrictionIndexCase(
          32, ConditioningClass::MildlyIllConditioned, 27032, true),
      makeFrictionIndexCase(
          48, ConditioningClass::MildlyIllConditioned, 27048, true),
      makeFrictionIndexCase(
          64, ConditioningClass::MildlyIllConditioned, 27064, true),
      makeFrictionIndexCase(
          96, ConditioningClass::MildlyIllConditioned, 27096, true),
      makeFrictionIndexCase(
          6, ConditioningClass::MildlyIllConditioned, 28006, true, 4.0),
      makeFrictionIndexCase(
          8, ConditioningClass::MildlyIllConditioned, 28008, true, 4.0),
      makeFrictionIndexCase(
          12, ConditioningClass::MildlyIllConditioned, 28012, true, 4.0),
      makeFrictionIndexCase(
          16, ConditioningClass::MildlyIllConditioned, 28016, true, 4.0),
      makeFrictionIndexCase(
          24, ConditioningClass::MildlyIllConditioned, 28024, true, 4.0),
      makeFrictionIndexCase(
          32, ConditioningClass::MildlyIllConditioned, 28032, true, 4.0),
      makeFrictionIndexCase(
          48, ConditioningClass::MildlyIllConditioned, 28048, true, 4.0),
      makeFrictionIndexCase(
          64, ConditioningClass::MildlyIllConditioned, 28064, true, 4.0),
      makeFrictionIndexCase(
          96, ConditioningClass::MildlyIllConditioned, 28096, true, 4.0),
      makeFrictionIndexCase(
          6, ConditioningClass::MildlyIllConditioned, 29006, true, 8.0),
      makeFrictionIndexCase(
          8, ConditioningClass::MildlyIllConditioned, 29008, true, 8.0),
      makeFrictionIndexCase(
          12, ConditioningClass::MildlyIllConditioned, 29012, true, 8.0),
      makeFrictionIndexCase(
          16, ConditioningClass::MildlyIllConditioned, 29016, true, 8.0),
      makeFrictionIndexCase(
          24, ConditioningClass::MildlyIllConditioned, 29024, true, 8.0),
      makeFrictionIndexCase(
          32, ConditioningClass::MildlyIllConditioned, 29032, true, 8.0),
      makeFrictionIndexCase(
          48, ConditioningClass::MildlyIllConditioned, 29048, true, 8.0),
      makeFrictionIndexCase(
          64, ConditioningClass::MildlyIllConditioned, 29064, true, 8.0),
      makeFrictionIndexCase(
          96, ConditioningClass::MildlyIllConditioned, 29096, true, 8.0),
      makeFrictionIndexCase(
          6, ConditioningClass::MildlyIllConditioned, 30006, true, 16.0),
      makeFrictionIndexCase(
          8, ConditioningClass::MildlyIllConditioned, 30008, true, 16.0),
      makeFrictionIndexCase(
          12, ConditioningClass::MildlyIllConditioned, 30012, true, 16.0),
      makeFrictionIndexCase(
          16, ConditioningClass::MildlyIllConditioned, 30016, true, 16.0),
      makeFrictionIndexCase(
          24, ConditioningClass::MildlyIllConditioned, 30024, true, 16.0),
      makeFrictionIndexCase(
          32, ConditioningClass::MildlyIllConditioned, 30032, true, 16.0),
      makeFrictionIndexCase(
          48, ConditioningClass::MildlyIllConditioned, 30048, true, 16.0),
      makeFrictionIndexCase(
          64, ConditioningClass::MildlyIllConditioned, 30064, true, 16.0),
      makeFrictionIndexCase(
          96, ConditioningClass::MildlyIllConditioned, 30096, true, 16.0),
      makeFrictionIndexCase(
          128, ConditioningClass::MildlyIllConditioned, 30128, true, 16.0),
      makeFrictionIndexCase(
          192, ConditioningClass::MildlyIllConditioned, 30192, true, 16.0)};
}

std::vector<GeneratedCase> makeRobustNearSingularCases()
{
  return {
      makeStandardCase(8, ConditioningClass::NearSingular, true, 14008),
      makeBoxedCase(8, ConditioningClass::NearSingular, 15008),
      makeFrictionIndexCase(3, ConditioningClass::NearSingular, 16003, true),
      makeFrictionIndexCase(6, ConditioningClass::NearSingular, 16006, true),
      makeFrictionIndexCase(9, ConditioningClass::NearSingular, 16009, true),
      makeFrictionIndexCase(12, ConditioningClass::NearSingular, 16012, true),
      makeFrictionIndexCase(16, ConditioningClass::NearSingular, 16016, true),
      makeFrictionIndexCase(24, ConditioningClass::NearSingular, 16024, true),
      makeFrictionIndexCase(32, ConditioningClass::NearSingular, 16032, true),
      makeFrictionIndexCase(48, ConditioningClass::NearSingular, 16048, true),
      makeFrictionIndexCase(64, ConditioningClass::NearSingular, 16064, true),
      makeFrictionIndexCase(
          96, ConditioningClass::NearSingular, 16096, true, 1.0, 0.08, 1e3),
      makeFrictionIndexCase(
          128, ConditioningClass::NearSingular, 16128, true, 1.0, 0.08, 1e3),
      makeFrictionIndexCase(
          192, ConditioningClass::NearSingular, 16192, true, 1.0, 0.08, 1e3),
      makeFrictionIndexCase(
          256, ConditioningClass::NearSingular, 16256, true, 1.0, 0.08, 1e3)};
}

std::vector<GeneratedCase> makeSingularDegenerateCases()
{
  return {
      makeSingularDegenerateStandardCase(16),
      makeSingularDegenerateBoxedCase(16),
      makeSingularDegenerateFrictionIndexCase(6)};
}

std::vector<GeneratedCase> makeLargerSingularDegenerateCases()
{
  return {
      makeSingularDegenerateStandardCase(32),
      makeSingularDegenerateBoxedCase(32),
      makeSingularDegenerateFrictionIndexCase(8)};
}

std::vector<GeneratedCase> makeStressSingularDegenerateCases()
{
  return {
      makeSingularDegenerateStandardCase(64),
      makeSingularDegenerateBoxedCase(64),
      makeSingularDegenerateFrictionIndexCase(12)};
}

#ifndef DART_LCP_GENERATED_COVERAGE_SKIP_EXTREME_SINGULAR_DEGENERATE
std::vector<GeneratedCase> makeExtremeSingularDegenerateCases()
{
  return {
      makeSingularDegenerateStandardCase(128),
      makeSingularDegenerateBoxedCase(128),
      makeSingularDegenerateFrictionIndexCase(16),
      makeSingularDegenerateFrictionIndexCase(24),
      makeSingularDegenerateFrictionIndexCase(32),
      makeSingularDegenerateFrictionIndexCase(48),
      makeSingularDegenerateFrictionIndexCase(64),
      makeSingularDegenerateFrictionIndexCase(96),
      makeSingularDegenerateFrictionIndexCase(128),
      makeSingularDegenerateFrictionIndexCase(192),
      makeSingularDegenerateFrictionIndexCase(256)};
}
#endif

std::vector<LcpProblem> makeInvalidProblems()
{
  std::vector<LcpProblem> problems;

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(3);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo(2);
    lo << 0.0, 2.0;
    Eigen::VectorXd hi(2);
    hi << 1.0, 1.0;
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
    Eigen::VectorXi findex(2);
    findex << -1, 4;
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
    hi[1] = std::numeric_limits<double>::quiet_NaN();
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    lo[1] = std::numeric_limits<double>::infinity();
    Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd hi = Eigen::VectorXd::Ones(2);
    hi[1] = -std::numeric_limits<double>::infinity();
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    problems.emplace_back(A, b, lo, hi, findex);
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd lo(2);
    lo << 0.0, -1.0;
    Eigen::VectorXd hi(2);
    hi << std::numeric_limits<double>::infinity(), -0.5;
    Eigen::VectorXi findex(2);
    findex << -1, 0;
    problems.emplace_back(A, b, lo, hi, findex);
  }

  return problems;
}

bool solverSupportsConcreteProblem(
    const LcpSolverManifestEntry& solverEntry, const LcpProblem& problem)
{
  const auto solver = solverEntry.create();
  return solver != nullptr && solver->supportsProblem(problem);
}

bool solverShouldRun(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  return solverSupportsConcreteProblem(solver, testCase.problem);
}

template <std::size_t N>
bool solverNameIn(
    const LcpSolverManifestEntry& solver,
    const std::array<std::string_view, N>& names)
{
  return std::find(names.begin(), names.end(), solver.name) != names.end();
}

bool solverShouldRunLargerCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  constexpr std::array<std::string_view, 17> kLargerCaseSolvers{{
      "Dantzig",
      "Pgs",
      "SymmetricPsor",
      "Jacobi",
      "RedBlackGaussSeidel",
      "BlockedJacobi",
      "BGS",
      "NNCG",
      "SubspaceMinimization",
      "Apgd",
      "Tgs",
      "MPRGP",
      "ShockPropagation",
      "Staggering",
      "Admm",
      "Sap",
      "BoxedSemiSmoothNewton",
  }};

  return solverNameIn(solver, kLargerCaseSolvers);
}

bool solverShouldRunProductionScaleCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  constexpr std::array<std::string_view, 14> kProductionScaleSolvers{{
      "Pgs",
      "SymmetricPsor",
      "Jacobi",
      "RedBlackGaussSeidel",
      "BlockedJacobi",
      "BGS",
      "NNCG",
      "SubspaceMinimization",
      "Apgd",
      "Tgs",
      "ShockPropagation",
      "Staggering",
      "Admm",
      "Sap",
  }};

  if (testCase.family == GeneratedFamily::Standard && solver.name == "MPRGP") {
    return true;
  }

  return solverNameIn(solver, kProductionScaleSolvers);
}

bool solverShouldRunLargerMildlyIllConditionedCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  // These cases assert the selected generated solution, not just the LCP
  // contract. Solvers that converge to alternative valid solutions stay covered
  // by the benchmark contract rows instead.
  constexpr std::array<std::string_view, 6> kLargerMildExactSolvers{{
      "Pgs",
      "NNCG",
      "Apgd",
      "Tgs",
      "Admm",
      "Sap",
  }};

  if (testCase.family == GeneratedFamily::Standard) {
    constexpr std::array<std::string_view, 2> kExactStandardSolvers{{
        "Dantzig",
        "Baraff",
    }};
    return solverNameIn(solver, kExactStandardSolvers)
           || solverNameIn(solver, kLargerMildExactSolvers);
  }

  if (testCase.family == GeneratedFamily::FrictionIndex && testCase.coupled
      && testCase.couplingScale > 8.0) {
    return solverNameIn(solver, kLargerMildExactSolvers)
           || solver.name == "BoxedSemiSmoothNewton";
  }

  if (testCase.family == GeneratedFamily::FrictionIndex && testCase.coupled
      && solver.name == "BoxedSemiSmoothNewton") {
    return true;
  }

  return solverNameIn(solver, kLargerMildExactSolvers);
}

bool solverShouldRunRobustNearSingularCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  if (testCase.family == GeneratedFamily::Standard) {
    constexpr std::array<std::string_view, 2> kStandardSolvers{{
        "Dantzig",
        "Baraff",
    }};
    return solverNameIn(solver, kStandardSolvers);
  }

  if (testCase.family == GeneratedFamily::Boxed) {
    constexpr std::array<std::string_view, 3> kBoxedSolvers{{
        "Dantzig",
        "ShockPropagation",
        "BoxedSemiSmoothNewton",
    }};
    return solverNameIn(solver, kBoxedSolvers);
  }

  constexpr std::array<std::string_view, 1> kFrictionSolvers{{
      "Dantzig",
  }};
  return solverNameIn(solver, kFrictionSolvers);
}

bool solverShouldRunSingularDegenerateCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  if (testCase.family == GeneratedFamily::Standard) {
    constexpr std::array<std::string_view, 21> kStandardSolvers{{
        "Dantzig",
        "Lemke",
        "Baraff",
        "Pgs",
        "SymmetricPsor",
        "Jacobi",
        "RedBlackGaussSeidel",
        "BlockedJacobi",
        "BGS",
        "NNCG",
        "SubspaceMinimization",
        "Apgd",
        "Tgs",
        "MinimumMapNewton",
        "FischerBurmeisterNewton",
        "InteriorPoint",
        "MPRGP",
        "ShockPropagation",
        "Staggering",
        "Admm",
        "Sap",
    }};
    return solverNameIn(solver, kStandardSolvers);
  }

  if (testCase.family == GeneratedFamily::Boxed) {
    constexpr std::array<std::string_view, 3> kBoxedSolvers{{
        "Admm",
        "Sap",
        "BoxedSemiSmoothNewton",
    }};
    return solverNameIn(solver, kBoxedSolvers);
  }

  constexpr std::array<std::string_view, 3> kFrictionSolvers{{
      "Admm",
      "Sap",
      "BoxedSemiSmoothNewton",
  }};
  return solverNameIn(solver, kFrictionSolvers);
}

void expectSolverPassesGeneratedCase(
    const LcpSolverManifestEntry& solverEntry, const GeneratedCase& testCase)
{
  auto solver = solverEntry.create();
  SolverRunConfig config(solverEntry.name, testCase);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(testCase.problem.b.size());
  const auto report
      = dart::test::SolveAndCheck(*solver, testCase.problem, x, config.options);

  const bool statusOk
      = report.result.succeeded()
        || (config.allowMaxIterations
            && report.result.status == LcpSolverStatus::MaxIterations
            && report.check.ok);

  EXPECT_TRUE(statusOk) << solverEntry.name << " on " << testCase.name << ": "
                        << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok) << solverEntry.name << " on " << testCase.name
                               << ": " << dart::test::DescribeReport(report);

  ASSERT_EQ(x.size(), testCase.expected.size())
      << solverEntry.name << " on " << testCase.name;
  const double error = (x - testCase.expected).lpNorm<Eigen::Infinity>();
  EXPECT_NEAR(error, 0.0, config.expectedTolerance)
      << solverEntry.name << " on " << testCase.name << ": "
      << dart::test::DescribeReport(report);
}

void runGeneratedCasesForSupportingSolvers(
    const std::vector<GeneratedCase>& cases)
{
  for (const auto& testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRun(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

bool solverShouldRunCoupledFrictionIndexKnownSolutionCase(
    const LcpSolverManifestEntry& solver, const GeneratedCase& testCase)
{
  if (!solverShouldRun(solver, testCase)) {
    return false;
  }

  if (testCase.conditioning != ConditioningClass::MildlyIllConditioned) {
    return true;
  }

  // These cases assert the selected generated solution, not just any valid
  // LCP solution. Solvers that converge to alternate valid mildly coupled
  // friction-index solutions stay covered by benchmark contract rows.
  constexpr std::array<std::string_view, 8> kMildCoupledExactSolvers{{
      "Dantzig",
      "Pgs",
      "NNCG",
      "Apgd",
      "Tgs",
      "Admm",
      "Sap",
      "BoxedSemiSmoothNewton",
  }};
  return solverNameIn(solver, kMildCoupledExactSolvers);
}

void expectSolverRejectsInvalidProblem(
    const LcpSolverManifestEntry& solverEntry, const LcpProblem& problem)
{
  auto solver = solverEntry.create();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  const auto result = solver->solve(problem, x, solver->getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem)
      << solverEntry.name << " unexpectedly returned "
      << dart::math::toString(result.status) << " message=" << result.message;
  EXPECT_FALSE(result.message.empty()) << solverEntry.name;
}

const LcpSolverManifestEntry* findSolver(const std::string_view name)
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (solver.name == name) {
      return &solver;
    }
  }

  return nullptr;
}

} // namespace

//==============================================================================
TEST(LcpGeneratedCoverage, StandardKnownSolutionsAcrossSizesAndConditioning)
{
  runGeneratedCasesForSupportingSolvers(makeStandardCases());
}

//==============================================================================
TEST(LcpGeneratedCoverage, BoxedKnownSolutionsAcrossSizesAndActiveSets)
{
  runGeneratedCasesForSupportingSolvers(makeBoxedCases());
}

//==============================================================================
TEST(LcpGeneratedCoverage, FrictionIndexKnownSolutionsAcrossContactCounts)
{
  runGeneratedCasesForSupportingSolvers(makeFrictionIndexCases());
}

//==============================================================================
TEST(
    LcpGeneratedCoverage, CoupledFrictionIndexKnownSolutionsAcrossContactCounts)
{
  for (const auto& testCase : makeCoupledFrictionIndexCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunCoupledFrictionIndexKnownSolutionCase(
              solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(LcpGeneratedCoverage, ActiveSetTransitionKnownSolutions)
{
  runGeneratedCasesForSupportingSolvers(makeActiveSetTransitionCases());
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    LargerActiveSetTransitionKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeLargerActiveSetTransitionCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    StressActiveSetTransitionKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeStressActiveSetTransitionCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    ExtremeActiveSetTransitionKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeExtremeActiveSetTransitionCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    ProductionActiveSetTransitionFrictionIndexKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeProductionActiveSetTransitionCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(LcpGeneratedCoverage, BatchShapedKnownSolutionsDoNotCarryState)
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    for (const auto& testCase : makeBatchCases()) {
      if (!solverShouldRun(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(LcpGeneratedCoverage, NearSingularStandardKnownSolutionsForPivotingSolvers)
{
  constexpr std::array<std::string_view, 4> kPivotingSolvers{
      "Dantzig", "Lemke", "Baraff", "Direct"};

  for (const auto solverName : kPivotingSolvers) {
    const auto* solver = findSolver(solverName);
    ASSERT_NE(solver, nullptr) << solverName;

    for (const auto& testCase : makeNearSingularStandardCases()) {
      if (!solverShouldRun(*solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(*solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage, LargerWellConditionedKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeLargerWellConditionedCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    ProductionScaleWellConditionedKnownSolutionsForScalableSolvers)
{
  for (const auto& testCase : makeProductionScaleWellConditionedCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunProductionScaleCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    LargerMildlyIllConditionedKnownSolutionsForScopedSolvers)
{
  for (const auto& testCase : makeLargerMildlyIllConditionedCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunLargerMildlyIllConditionedCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(LcpGeneratedCoverage, NearSingularKnownSolutionsForRobustSolverSlice)
{
  for (const auto& testCase : makeRobustNearSingularCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunRobustNearSingularCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(LcpGeneratedCoverage, SingularDegenerateKnownSolutionsForRobustSolverSlice)
{
  for (const auto& testCase : makeSingularDegenerateCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunSingularDegenerateCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    LargerSingularDegenerateKnownSolutionsForRobustSolverSlice)
{
  for (const auto& testCase : makeLargerSingularDegenerateCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunSingularDegenerateCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

//==============================================================================
TEST(
    LcpGeneratedCoverage,
    StressSingularDegenerateKnownSolutionsForRobustSolverSlice)
{
  for (const auto& testCase : makeStressSingularDegenerateCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunSingularDegenerateCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}

// Code coverage instrumentation and Debug assertions make this largest
// rank-deficient slice exceed the CI per-test timeout. Optimized CI still runs
// it without DART_LCP_GENERATED_COVERAGE_SKIP_EXTREME_SINGULAR_DEGENERATE.
#ifndef DART_LCP_GENERATED_COVERAGE_SKIP_EXTREME_SINGULAR_DEGENERATE
//==============================================================================
TEST(
    LcpGeneratedCoverage,
    ExtremeSingularDegenerateKnownSolutionsForRobustSolverSlice)
{
  for (const auto& testCase : makeExtremeSingularDegenerateCases()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!solverShouldRunSingularDegenerateCase(solver, testCase)) {
        continue;
      }
      expectSolverPassesGeneratedCase(solver, testCase);
    }
  }
}
#endif

//==============================================================================
TEST(LcpGeneratedCoverage, ThreadedJacobiStandardKnownSolution)
{
  const auto testCase
      = makeStandardCase(128, ConditioningClass::WellConditioned, true, 17128);
  dart::math::JacobiSolver solver;
  SolverRunConfig config("Jacobi", testCase);
  dart::math::JacobiSolver::Parameters params;
  params.workerThreads = 4;
  config.options.customOptions = &params;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(testCase.problem.b.size());
  const auto report
      = dart::test::SolveAndCheck(solver, testCase.problem, x, config.options);

  EXPECT_TRUE(report.result.succeeded()) << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok) << dart::test::DescribeReport(report);
  ASSERT_EQ(x.size(), testCase.expected.size());
  const double error = (x - testCase.expected).lpNorm<Eigen::Infinity>();
  EXPECT_NEAR(error, 0.0, config.expectedTolerance)
      << dart::test::DescribeReport(report);
}

//==============================================================================
TEST(LcpGeneratedCoverage, ThreadedRedBlackGaussSeidelStandardKnownSolution)
{
  const auto testCase
      = makeStandardCase(128, ConditioningClass::WellConditioned, true, 19128);
  dart::math::RedBlackGaussSeidelSolver solver;
  SolverRunConfig config("RedBlackGaussSeidel", testCase);
  dart::math::RedBlackGaussSeidelSolver::Parameters params;
  params.workerThreads = 4;
  config.options.customOptions = &params;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(testCase.problem.b.size());
  const auto report
      = dart::test::SolveAndCheck(solver, testCase.problem, x, config.options);

  EXPECT_TRUE(report.result.succeeded()) << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok) << dart::test::DescribeReport(report);
  ASSERT_EQ(x.size(), testCase.expected.size());
  const double error = (x - testCase.expected).lpNorm<Eigen::Infinity>();
  EXPECT_NEAR(error, 0.0, config.expectedTolerance)
      << dart::test::DescribeReport(report);
}

//==============================================================================
TEST(LcpGeneratedCoverage, ThreadedBlockedJacobiStandardKnownSolution)
{
  const auto testCase
      = makeStandardCase(128, ConditioningClass::WellConditioned, true, 20128);
  dart::math::BlockedJacobiSolver solver;
  SolverRunConfig config("BlockedJacobi", testCase);
  dart::math::BlockedJacobiSolver::Parameters params;
  params.workerThreads = 4;
  config.options.customOptions = &params;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(testCase.problem.b.size());
  const auto report
      = dart::test::SolveAndCheck(solver, testCase.problem, x, config.options);

  EXPECT_TRUE(report.result.succeeded()) << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok) << dart::test::DescribeReport(report);
  ASSERT_EQ(x.size(), testCase.expected.size());
  const double error = (x - testCase.expected).lpNorm<Eigen::Infinity>();
  EXPECT_NEAR(error, 0.0, config.expectedTolerance)
      << dart::test::DescribeReport(report);
}

//==============================================================================
TEST(LcpGeneratedCoverage, InvalidProblemsReportInvalidProblem)
{
  for (const auto& problem : makeInvalidProblems()) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      expectSolverRejectsInvalidProblem(solver, problem);
    }
  }
}
