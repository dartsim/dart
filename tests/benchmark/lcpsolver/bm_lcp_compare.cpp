/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Solver-agnostic benchmark harness for LCP comparisons.
 */

#include "tests/common/lcpsolver/lcp_problem_factory.hpp"
#include "tests/common/lcpsolver/lcp_solver_manifest.hpp"
#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#ifndef DART_BM_LCP_COMPARE_HAS_SIMULATION
  #define DART_BM_LCP_COMPARE_HAS_SIMULATION 0
#endif
#ifndef DART_BM_LCP_COMPARE_DART_ENABLE_SIMD
  #define DART_BM_LCP_COMPARE_DART_ENABLE_SIMD 0
#endif
#ifndef DART_BM_LCP_COMPARE_DART_SIMD_FORCE_SCALAR
  #define DART_BM_LCP_COMPARE_DART_SIMD_FORCE_SCALAR 0
#endif
#ifndef DART_BM_LCP_COMPARE_DART_ENABLE_EXPERIMENTAL_CUDA
  #define DART_BM_LCP_COMPARE_DART_ENABLE_EXPERIMENTAL_CUDA 0
#endif
#ifndef DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
  #define DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA 0
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
  #include <dart/simulation/body/collision_shape.hpp>
  #include <dart/simulation/body/contact.hpp>
  #include <dart/simulation/body/rigid_body.hpp>
  #include <dart/simulation/body/rigid_body_options.hpp>
  #include <dart/simulation/comps/multibody.hpp>
  #include <dart/simulation/compute/compute_graph.hpp>
  #include <dart/simulation/compute/multibody_dynamics.hpp>
  #include <dart/simulation/compute/parallel_executor.hpp>
  #include <dart/simulation/compute/unified_constraint.hpp>
  #include <dart/simulation/detail/boxed_lcp_contact.hpp>
  #include <dart/simulation/detail/entity_conversion.hpp>
  #include <dart/simulation/detail/world_registry_access.hpp>
  #include <dart/simulation/multibody/joint.hpp>
  #include <dart/simulation/multibody/link.hpp>
  #include <dart/simulation/multibody/multibody.hpp>
  #include <dart/simulation/world.hpp>
  #include <dart/simulation/world_options.hpp>
#endif
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
  #include <dart/simulation/compute/cuda/lcp_jacobi_batch_cuda.cuh>
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <cmath>
#include <cstddef>

static_assert(dart::test::kLcpSolverManifest.size() == 24);
static_assert(
    dart::test::countSolversSupporting(dart::test::LcpProblemSupport::Standard)
    == 23);
static_assert(
    dart::test::countSolversSupporting(dart::test::LcpProblemSupport::Boxed)
    == 15);
static_assert(
    dart::test::countSolversSupporting(
        dart::test::LcpProblemSupport::FrictionIndex)
    == 16);

namespace {

using dart::math::LcpOptions;
using dart::math::LcpProblem;
#if DART_BM_LCP_COMPARE_HAS_SIMULATION
namespace compute = dart::simulation::compute;
namespace sx = dart::simulation;
#endif
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
namespace cuda_compute = dart::simulation::compute::cuda;
#endif

double MakeDenseBoxGroundHalfExtent(int boxCount)
{
  // Keep larger positive-quadrant box grids fully supported by the ground.
  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  const int rows = (boxCount + columns - 1) / columns;
  constexpr double kSpacing = 2.0;
  return std::max(
      20.0, kSpacing * static_cast<double>(std::max(columns, rows) - 1) + 1.0);
}

double MakePositiveGridGroundHalfExtent(
    int itemCount, double spacing, double minimumHalfExtent)
{
  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(itemCount))));
  const int rows = (itemCount + columns - 1) / columns;
  return std::max(
      minimumHalfExtent,
      spacing * static_cast<double>(std::max(columns, rows) - 1) + 1.0);
}

LcpOptions MakeBenchmarkOptions(int maxIterations)
{
  LcpOptions options;
  options.maxIterations = maxIterations;
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;
  options.relaxation = 1.0;
  options.warmStart = false;
  options.validateSolution = false;
  options.earlyTermination = false;
  return options;
}

LcpProblem MakeStandardSpdProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (const int r : std::views::iota(0, n)) {
    for (const int c : std::views::iota(0, n)) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd xStar(n);
  for (const int i : std::views::iota(0, n)) {
    xStar[i] = std::abs(dist(rng)) + 0.1;
  }

  Eigen::VectorXd b = A * xStar;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem MakeStandardBandedSpdProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (const int i : std::views::iota(0, n)) {
    A(i, i) = 6.0;
    if (i + 1 < n) {
      A(i, i + 1) = -1.0;
      A(i + 1, i) = -1.0;
    }
    if (i + 2 < n) {
      A(i, i + 2) = -0.25;
      A(i + 2, i) = -0.25;
    }
  }

  Eigen::VectorXd xStar(n);
  for (const int i : std::views::iota(0, n)) {
    xStar[i] = std::abs(dist(rng)) + 0.25;
  }

  Eigen::VectorXd b = A * xStar;
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi
      = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem MakeBoxedActiveBoundsProblem(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> slackDist(0.1, 1.0);

  Eigen::MatrixXd M(n, n);
  for (const int r : std::views::iota(0, n)) {
    for (const int c : std::views::iota(0, n)) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd lo = Eigen::VectorXd::Constant(n, -1.0);
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, 1.0);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

  Eigen::VectorXd xStar(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  for (const int i : std::views::iota(0, n)) {
    const int mode = i % 3;
    if (mode == 0) {
      xStar[i] = lo[i];
      w[i] = slackDist(rng);
    } else if (mode == 1) {
      xStar[i] = hi[i];
      w[i] = -slackDist(rng);
    } else {
      xStar[i] = 0.5 * dist(rng);
      w[i] = 0.0;
    }
  }

  Eigen::VectorXd b = A * xStar - w;

  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem MakeFrictionIndexProblem(int numContacts, unsigned seed)
{
  const int n = 3 * numContacts;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> muDist(0.2, 1.0);

  Eigen::MatrixXd M(n, n);
  for (const int r : std::views::iota(0, n)) {
    for (const int c : std::views::iota(0, n)) {
      M(r, c) = dist(rng);
    }
  }

  Eigen::MatrixXd A
      = M.transpose() * M
        + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd xStar = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

  for (const int c : std::views::iota(0, numContacts)) {
    const int base = 3 * c;
    const double normal = std::abs(dist(rng)) + 0.5;
    const double mu = muDist(rng);

    xStar[base + 0] = normal;
    xStar[base + 1] = 0.5 * mu * normal;
    xStar[base + 2] = -0.25 * mu * normal;

    lo[base + 0] = 0.0;
    hi[base + 0] = std::numeric_limits<double>::infinity();

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base + 0;

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base + 0;
  }

  Eigen::VectorXd b = A * xStar;
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

double BenchmarkSignedUnitValue(const int i, const int j, const unsigned seed)
{
  const unsigned value = static_cast<unsigned>(
      37 * (i + 1) + 101 * (j + 3) + 17 * static_cast<int>(seed));
  const double scaled = static_cast<double>(value % 19) / 9.0 - 1.0;
  return scaled == 0.0 ? 0.25 : scaled;
}

Eigen::MatrixXd MakeActiveSetTransitionSpd(const int n, const unsigned seed)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int i = 0; i < n; ++i) {
    diag[i] = 1.0 + 0.15 * static_cast<double>((seed + i) % 7);
  }

  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const double magnitude
          = 0.01 * std::min(diag[r], diag[c]) / std::max(1, n - 1);
      const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }

  for (int r = 0; r < n; ++r) {
    A(r, r) = diag[r] + A.row(r).cwiseAbs().sum() + 0.1;
  }

  return A;
}

LcpProblem MakeStandardActiveSetTransitionProblem(
    const int n = 16, const unsigned seed = 18'016u)
{
  Eigen::MatrixXd A = MakeActiveSetTransitionSpd(n, seed);
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
  const double inf = std::numeric_limits<double>::infinity();
  return LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, inf),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeBoxedActiveSetTransitionProblem(
    const int n = 16, const unsigned seed = 19'016u)
{
  Eigen::MatrixXd A = MakeActiveSetTransitionSpd(n, seed);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeFrictionIndexActiveSetTransitionProblem(
    const int numContacts = 6,
    const unsigned seed = 20'006u,
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
      const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }
  for (int r = 0; r < n; ++r) {
    A(r, r) = diag[r] + A.row(r).cwiseAbs().sum() + 0.1;
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem MakeActiveFrictionIndexContactProblem()
{
  auto factoryProblem
      = dart::test::LcpProblemFactory::activeFrictionIndexContact();
  return std::move(factoryProblem.problem);
}

Eigen::MatrixXd MakeMildIllConditionedSpd(const int n, const unsigned seed)
{
  Eigen::VectorXd baseDiag(n);
  constexpr double maxScale = 250.0;
  for (int i = 0; i < n; ++i) {
    const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    baseDiag[i] = 1.0 + std::pow(maxScale, t);
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const double magnitude
          = 0.0125 * std::min(baseDiag[r], baseDiag[c]) / std::max(1, n - 1);
      const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }

  for (int r = 0; r < n; ++r) {
    const double offDiagonalSum = A.row(r).cwiseAbs().sum();
    A(r, r) = baseDiag[r] + offDiagonalSum + 0.25;
  }

  return A;
}

LcpProblem MakeMildIllConditionedStandardProblem(
    const int n, const unsigned seed)
{
  Eigen::MatrixXd A = MakeMildIllConditionedSpd(n, seed);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    if (i % 4 == 0) {
      w[i] = 0.2 + 0.03 * static_cast<double>(i + 1);
    } else {
      x[i] = 0.1 + 0.04 * static_cast<double>((i % 5) + 1);
    }
  }

  Eigen::VectorXd b = A * x - w;
  const double inf = std::numeric_limits<double>::infinity();
  return LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, inf),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeMildIllConditionedBoxedProblem(const int n, const unsigned seed)
{
  Eigen::MatrixXd A = MakeMildIllConditionedSpd(n, seed);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeMildIllConditionedFrictionIndexProblem(
    const int numContacts,
    const unsigned seed,
    const bool coupled,
    const double couplingScale = 1.0)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int contact = 0; contact < numContacts; ++contact) {
    const double contactScale
        = 1.0 + 0.1 * static_cast<double>((seed + contact) % 7);
    diag[3 * contact + 0] = contactScale;
    diag[3 * contact + 1] = contactScale * 25.0;
    diag[3 * contact + 2] = contactScale * 25.0 * 25.0;
  }

  if (coupled) {
    for (int r = 0; r < n; ++r) {
      for (int c = r + 1; c < n; ++c) {
        const bool sameContact = (r / 3) == (c / 3);
        const double strength = couplingScale * (sameContact ? 0.025 : 0.01);
        const double magnitude
            = strength * std::min(diag[r], diag[c]) / std::max(1, n - 1);
        const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
        A(r, c) = value;
        A(c, r) = value;
      }
    }
  }

  for (int r = 0; r < n; ++r) {
    A(r, r) = diag[r] + (coupled ? A.row(r).cwiseAbs().sum() : 0.0);
  }

  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int contact = 0; contact < numContacts; ++contact) {
    const int base = 3 * contact;
    const double normal = 0.5 + 0.15 * static_cast<double>(contact + 1);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

Eigen::MatrixXd MakeNearSingularSpd(const int n, const unsigned seed)
{
  Eigen::VectorXd baseDiag(n);
  constexpr double maxScale = 1e6;
  for (int i = 0; i < n; ++i) {
    const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    baseDiag[i] = std::pow(maxScale, t) / maxScale;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const double magnitude
          = 0.0125 * std::min(baseDiag[r], baseDiag[c]) / std::max(1, n - 1);
      const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
      A(r, c) = value;
      A(c, r) = value;
    }
  }

  for (int r = 0; r < n; ++r) {
    A(r, r) = baseDiag[r] + A.row(r).cwiseAbs().sum() + 1e-9;
  }

  return A;
}

LcpProblem MakeNearSingularStandardProblem(const int n, const unsigned seed)
{
  Eigen::MatrixXd A = MakeNearSingularSpd(n, seed);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    if (i % 4 == 0) {
      w[i] = 0.2 + 0.03 * static_cast<double>(i + 1);
    } else {
      x[i] = 0.1 + 0.04 * static_cast<double>((i % 5) + 1);
    }
  }

  Eigen::VectorXd b = A * x - w;
  const double inf = std::numeric_limits<double>::infinity();
  return LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, inf),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeNearSingularBoxedProblem(const int n, const unsigned seed)
{
  Eigen::MatrixXd A = MakeNearSingularSpd(n, seed);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeNearSingularFrictionIndexProblem(
    const int numContacts,
    const unsigned seed,
    const bool coupled,
    const double normalSlope = 0.15,
    const double nearSingularScale = 1e4)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int contact = 0; contact < numContacts; ++contact) {
    const double contactScale
        = 1.0 + 0.1 * static_cast<double>((seed + contact) % 7);
    diag[3 * contact + 0] = contactScale;
    diag[3 * contact + 1] = contactScale * nearSingularScale;
    diag[3 * contact + 2]
        = contactScale * nearSingularScale * nearSingularScale;
  }

  if (coupled) {
    for (int r = 0; r < n; ++r) {
      for (int c = r + 1; c < n; ++c) {
        const bool sameContact = (r / 3) == (c / 3);
        const double strength = sameContact ? 0.025 : 0.01;
        const double magnitude
            = strength * std::min(diag[r], diag[c]) / std::max(1, n - 1);
        const double value = magnitude * BenchmarkSignedUnitValue(r, c, seed);
        A(r, c) = value;
        A(c, r) = value;
      }
    }
  }

  for (int r = 0; r < n; ++r) {
    A(r, r) = diag[r] + (coupled ? A.row(r).cwiseAbs().sum() : 0.0);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

LcpProblem MakeScaledProblem(int n, double scale, unsigned seed)
{
  auto problem = MakeStandardSpdProblem(n, seed);
  problem.A *= scale;
  problem.b *= scale;
  return problem;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
enum class ArticulatedContactBenchmarkCase
{
  Ground,
  RigidImpact,
  CrossLinkImpact
};

std::string_view getArticulatedContactBenchmarkCaseName(
    ArticulatedContactBenchmarkCase benchmarkCase)
{
  switch (benchmarkCase) {
    case ArticulatedContactBenchmarkCase::Ground:
      return "Ground";
    case ArticulatedContactBenchmarkCase::RigidImpact:
      return "RigidImpact";
    case ArticulatedContactBenchmarkCase::CrossLinkImpact:
      return "CrossLinkImpact";
  }

  return "Unknown";
}

struct WorldContactBenchmarkProblem
{
  LcpProblem problem;
  Eigen::VectorXd referenceSolution;
  std::size_t contactCount{0};
  std::size_t bodyCount{0};
};

struct WorldContactBenchmarkBatch
{
  std::vector<LcpProblem> problems;
  Eigen::Index totalProblemSize{0};
  std::size_t totalContactCount{0};
  std::size_t totalBodyCount{0};
};

enum class WorldContactBatchKind
{
  Baseline,
  StressStack,
  ContactPipeline32
};

enum class StaggeringContactPipelineKind
{
  WorldSeparated,
  WorldStack,
  ArticulatedGround,
  ArticulatedRigidImpact,
  ArticulatedCrossLinkImpact
};

struct StaggeringContactPipelineSweepCase
{
  StaggeringContactPipelineKind kind;
  int contactOrShapeCount;
  std::string_view caseLabel;
};

constexpr std::array<StaggeringContactPipelineSweepCase, 24>
    kStaggeringContactPipelineSweepCases{{
        {StaggeringContactPipelineKind::WorldSeparated, 1, "WorldSeparated1"},
        {StaggeringContactPipelineKind::WorldSeparated, 2, "WorldSeparated2"},
        {StaggeringContactPipelineKind::WorldSeparated, 4, "WorldSeparated4"},
        {StaggeringContactPipelineKind::WorldSeparated, 8, "WorldSeparated8"},
        {StaggeringContactPipelineKind::WorldSeparated, 16, "WorldSeparated16"},
        {StaggeringContactPipelineKind::WorldSeparated, 32, "WorldSeparated32"},
        {StaggeringContactPipelineKind::WorldStack, 2, "WorldStack2"},
        {StaggeringContactPipelineKind::WorldStack, 3, "WorldStack3"},
        {StaggeringContactPipelineKind::WorldStack, 5, "WorldStack5"},
        {StaggeringContactPipelineKind::WorldStack, 8, "WorldStack8"},
        {StaggeringContactPipelineKind::WorldStack, 16, "WorldStack16"},
        {StaggeringContactPipelineKind::WorldStack, 32, "WorldStack32"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         4,
         "ArticulatedGround4"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         8,
         "ArticulatedGround8"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         16,
         "ArticulatedGround16"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         32,
         "ArticulatedGround32"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         4,
         "ArticulatedRigidImpact4"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         8,
         "ArticulatedRigidImpact8"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         16,
         "ArticulatedRigidImpact16"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         32,
         "ArticulatedRigidImpact32"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         4,
         "ArticulatedCrossLinkImpact4"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         8,
         "ArticulatedCrossLinkImpact8"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         16,
         "ArticulatedCrossLinkImpact16"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         32,
         "ArticulatedCrossLinkImpact32"},
    }};

std::optional<WorldContactBenchmarkProblem> MakeWorldContactBenchmarkProblem(
    int contactCount, std::string& errorMessage, int velocityVariant = 0)
{
  if (contactCount <= 0) {
    errorMessage = "contact count must be positive";
    return std::nullopt;
  }

  constexpr double kFriction = 0.7;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(32.0, 32.0, 0.5)));
  ground.setFriction(kFriction);

  const int columns = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(contactCount))));
  const double spacing = 2.0;
  const double variantScale = static_cast<double>(velocityVariant);
  for (const int i : std::views::iota(0, contactCount)) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(
        spacing * static_cast<double>(col),
        spacing * static_cast<double>(row),
        0.5);
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.35 - 0.05 * static_cast<double>(i % 3) + 0.02 * variantScale,
        -0.25 + 0.04 * static_cast<double>(i % 5) - 0.015 * variantScale,
        -0.2 - 0.01 * variantScale);
    auto sphere
        = world.addRigidBody("sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  if (contacts.size() != static_cast<std::size_t>(contactCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " contacts, expected " + std::to_string(contactCount);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != static_cast<std::size_t>(contactCount)
      || snapshot.size() != static_cast<Eigen::Index>(3 * contactCount)) {
    errorMessage = "boxed-LCP contact snapshot shape did not match contacts";
    return std::nullopt;
  }

  return WorldContactBenchmarkProblem{
      LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.f,
      snapshot.contactCount,
      snapshot.bodyCount};
}

std::optional<WorldContactBenchmarkProblem> MakeWorldBoxContactBenchmarkProblem(
    std::string& errorMessage, int velocityVariant = 0, int boxCount = 1)
{
  if (boxCount <= 0) {
    errorMessage = "dense box contact box count must be positive";
    return std::nullopt;
  }

  constexpr double kFriction = 0.5;
  sx::WorldOptions options;
  options.timeStep = boxCount >= 24 ? 0.001 : 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  const double groundHalfExtent = MakeDenseBoxGroundHalfExtent(boxCount);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(groundHalfExtent, groundHalfExtent, 0.5)));
  ground.setFriction(kFriction);

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  constexpr double kSpacing = 2.0;
  const double variantScale = static_cast<double>(velocityVariant);
  for (const int i : std::views::iota(0, boxCount)) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions boxOptions;
    boxOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    boxOptions.linearVelocity = Eigen::Vector3d(
        0.35 + 0.02 * variantScale - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.015 * variantScale + 0.01 * static_cast<double>(i % 2),
        -0.02 - 0.002 * variantScale);
    auto box = world.addRigidBody("box_" + std::to_string(i), boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  const auto expectedContacts = static_cast<std::size_t>(4 * boxCount);
  if (contacts.size() != expectedContacts) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " box contacts, expected "
                   + std::to_string(expectedContacts);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != contacts.size()
      || snapshot.size()
             != static_cast<Eigen::Index>(3 * snapshot.contactCount)) {
    errorMessage
        = "boxed-LCP box contact snapshot shape did not match contacts";
    return std::nullopt;
  }
  if (snapshot.bodyCount != static_cast<std::size_t>(boxCount)) {
    errorMessage = "dense box contact snapshot body count changed";
    return std::nullopt;
  }

  return WorldContactBenchmarkProblem{
      LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.f,
      snapshot.contactCount,
      snapshot.bodyCount};
}

std::optional<WorldContactBenchmarkProblem>
MakeWorldStackContactBenchmarkProblem(
    int sphereCount, std::string& errorMessage, int velocityVariant = 0)
{
  if (sphereCount < 2) {
    errorMessage = "sphere count must be at least two for a contact stack";
    return std::nullopt;
  }

  constexpr double kFriction = 0.6;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(8.0, 8.0, 0.5)));
  ground.setFriction(kFriction);

  const double variantScale = static_cast<double>(velocityVariant);
  for (const int i : std::views::iota(0, sphereCount)) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position
        = Eigen::Vector3d(0.0, 0.0, 0.5 + static_cast<double>(i));
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.18 - 0.06 * static_cast<double>(i) + 0.01 * variantScale,
        -0.12 + 0.05 * static_cast<double>(i) - 0.008 * variantScale,
        -0.16 - 0.08 * static_cast<double>(i) - 0.005 * variantScale);
    auto sphere = world.addRigidBody(
        "stack_sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  if (contacts.size() != static_cast<std::size_t>(sphereCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " stack contacts, expected " + std::to_string(sphereCount);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != static_cast<std::size_t>(sphereCount)
      || snapshot.size() != static_cast<Eigen::Index>(3 * sphereCount)) {
    errorMessage
        = "boxed-LCP stack contact snapshot shape did not match contacts";
    return std::nullopt;
  }

  return WorldContactBenchmarkProblem{
      LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.f,
      snapshot.contactCount,
      snapshot.bodyCount};
}

struct ThreeAxisPrismaticBenchmarkRobot
{
  entt::entity multibody = entt::null;
  entt::entity link = entt::null;
};

ThreeAxisPrismaticBenchmarkRobot addThreeAxisPrismaticBenchmarkRobot(
    sx::World& world, const std::string& name)
{
  auto robot = world.addMultibody(name);
  auto base = robot.addLink("base");

  sx::JointSpec xSpec;
  xSpec.name = "x";
  xSpec.type = sx::JointType::Prismatic;
  xSpec.axis = Eigen::Vector3d::UnitX();
  auto xLink = robot.addLink("x_link", base, xSpec);
  xLink.setMass(1.0);
  xLink.setInertia(Eigen::Matrix3d::Identity());

  sx::JointSpec ySpec;
  ySpec.name = "y";
  ySpec.type = sx::JointType::Prismatic;
  ySpec.axis = Eigen::Vector3d::UnitY();
  auto yLink = robot.addLink("y_link", xLink, ySpec);
  yLink.setMass(1.0);
  yLink.setInertia(Eigen::Matrix3d::Identity());

  sx::JointSpec zSpec;
  zSpec.name = "z";
  zSpec.type = sx::JointType::Prismatic;
  zSpec.axis = Eigen::Vector3d::UnitZ();
  auto tip = robot.addLink("tip", yLink, zSpec);
  tip.setMass(1.0);
  tip.setInertia(Eigen::Matrix3d::Identity());

  return {
      sx::detail::toRegistryEntity(robot.getEntity()),
      sx::detail::toRegistryEntity(tip.getEntity())};
}

bool completeBenchmarkCrossMultibodyRows(
    sx::detail::WorldRegistry& registry,
    std::span<compute::UnifiedMultibodyContact> multibodyContacts,
    std::span<const Eigen::VectorXd> multibodyVelocities,
    std::string& errorMessage)
{
  if (multibodyVelocities.size() != multibodyContacts.size()) {
    errorMessage = "cross-multibody articulated velocity count changed";
    return false;
  }

  const auto findMultibodyIndex
      = [&](entt::entity multibody) -> std::optional<std::size_t> {
    for (std::size_t i = 0; i < multibodyContacts.size(); ++i) {
      if (multibodyContacts[i].multibody == multibody) {
        return i;
      }
    }
    return std::nullopt;
  };

  constexpr double restitutionThreshold = 1e-2;
  for (std::size_t primaryIndex = 0; primaryIndex < multibodyContacts.size();
       ++primaryIndex) {
    auto& primaryContact = multibodyContacts[primaryIndex];
    const Eigen::VectorXd& primaryVelocity = multibodyVelocities[primaryIndex];

    for (auto& row : primaryContact.problem.rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }

      const auto otherIndex = findMultibodyIndex(row.otherMultibody);
      if (!otherIndex.has_value()) {
        errorMessage = "cross-multibody articulated contact target is missing";
        return false;
      }

      const auto& otherStructure
          = registry.get<sx::comps::MultibodyStructure>(row.otherMultibody);
      const Eigen::MatrixXd otherWorldJacobian
          = compute::computeMultibodyLinkWorldJacobian(
              registry, otherStructure, row.otherLink);
      if (otherWorldJacobian.rows() != 6) {
        errorMessage = "cross-multibody articulated Jacobian shape changed";
        return false;
      }
      const Eigen::MatrixXd otherPointJacobian
          = otherWorldJacobian.bottomRows<3>();

      row.otherNormalJacobian = otherPointJacobian.transpose() * row.normal;
      row.otherTangentJacobian1 = otherPointJacobian.transpose() * row.tangent1;
      row.otherTangentJacobian2 = otherPointJacobian.transpose() * row.tangent2;

      const Eigen::MatrixXd& otherInverseMass
          = multibodyContacts[*otherIndex].problem.inverseMass;
      row.normalDenominator += row.otherNormalJacobian.dot(
          otherInverseMass * row.otherNormalJacobian);
      row.tangentDenominator1 += row.otherTangentJacobian1.dot(
          otherInverseMass * row.otherTangentJacobian1);
      row.tangentDenominator2 += row.otherTangentJacobian2.dot(
          otherInverseMass * row.otherTangentJacobian2);

      const Eigen::VectorXd& otherVelocity = multibodyVelocities[*otherIndex];
      const auto relativeVelocity = [&](const Eigen::VectorXd& primaryJacobian,
                                        const Eigen::VectorXd& otherJacobian) {
        return primaryJacobian.dot(primaryVelocity)
               - otherJacobian.dot(otherVelocity);
      };

      const double approachingVelocity
          = relativeVelocity(row.normalJacobian, row.otherNormalJacobian);
      row.restitutionTarget = (approachingVelocity < -restitutionThreshold)
                                  ? -row.restitution * approachingVelocity
                                  : 0.0;
      row.normalRhs
          = -approachingVelocity + std::max(row.bias, row.restitutionTarget);
      row.tangentRhs1
          = -relativeVelocity(row.tangentJacobian1, row.otherTangentJacobian1);
      row.tangentRhs2
          = -relativeVelocity(row.tangentJacobian2, row.otherTangentJacobian2);
      row.active = row.normalDenominator > 0.0;
    }
  }

  return true;
}

std::optional<WorldContactBenchmarkProblem>
MakeArticulatedUnifiedContactBenchmarkProblem(
    ArticulatedContactBenchmarkCase benchmarkCase,
    int contactCount,
    std::string& errorMessage,
    int velocityVariant = 0)
{
  if (contactCount <= 0) {
    errorMessage = "articulated contact count must be positive";
    return std::nullopt;
  }

  constexpr double kTimeStep = 0.005;
  constexpr double kFriction = 0.45;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto& registry = sx::detail::registryOf(world);
  std::vector<compute::UnifiedMultibodyContact> multibodyContacts;
  std::vector<Eigen::VectorXd> multibodyVelocities;
  multibodyContacts.reserve(static_cast<std::size_t>(2 * contactCount));
  multibodyVelocities.reserve(static_cast<std::size_t>(2 * contactCount));

  const double variantScale = static_cast<double>(velocityVariant);
  for (const int i : std::views::iota(0, contactCount)) {
    const auto robot = addThreeAxisPrismaticBenchmarkRobot(
        world, "three_axis_" + std::to_string(i));

    const Eigen::Vector3d point(
        1.5 * static_cast<double>(i),
        0.2 * static_cast<double>(i % 2),
        0.05 * static_cast<double>(i % 3));

    compute::LinkContact contact;
    contact.link = robot.link;
    contact.point = point;
    contact.depth = 0.004 + 0.0005 * static_cast<double>(i % 3);
    contact.friction = kFriction;
    contact.restitution = 0.0;

    Eigen::VectorXd nextVelocity(3);
    switch (benchmarkCase) {
      case ArticulatedContactBenchmarkCase::Ground:
        contact.normal = Eigen::Vector3d::UnitZ();
        nextVelocity << 0.35 + 0.02 * static_cast<double>(i % 3)
                            + 0.006 * variantScale,
            -0.25 + 0.015 * static_cast<double>(i % 2) - 0.004 * variantScale,
            -0.55 - 0.01 * static_cast<double>(i % 4) - 0.003 * variantScale;
        break;
      case ArticulatedContactBenchmarkCase::RigidImpact: {
        sx::RigidBodyOptions obstacleOptions;
        obstacleOptions.mass = 1.5;
        obstacleOptions.inertia = 0.8 * Eigen::Matrix3d::Identity();
        obstacleOptions.position = point;
        obstacleOptions.linearVelocity = Eigen::Vector3d(
            0.08 + 0.01 * static_cast<double>(i % 2), -0.03, 0.02);
        auto obstacle = world.addRigidBody(
            "dynamic_obstacle_" + std::to_string(i), obstacleOptions);

        contact.normal = Eigen::Vector3d::UnitX();
        contact.otherBody = sx::detail::toRegistryEntity(obstacle.getEntity());
        nextVelocity << -0.85 - 0.02 * static_cast<double>(i % 3)
                            - 0.004 * variantScale,
            0.22 - 0.015 * static_cast<double>(i % 2) + 0.003 * variantScale,
            -0.18 + 0.01 * static_cast<double>(i % 4) - 0.002 * variantScale;
        break;
      }
      case ArticulatedContactBenchmarkCase::CrossLinkImpact: {
        const auto target = addThreeAxisPrismaticBenchmarkRobot(
            world, "three_axis_target_" + std::to_string(i));
        contact.normal = Eigen::Vector3d::UnitX();
        contact.otherLink = target.link;
        contact.otherMultibody = target.multibody;
        nextVelocity << -0.85 - 0.02 * static_cast<double>(i % 3)
                            - 0.004 * variantScale,
            0.22 - 0.015 * static_cast<double>(i % 2) + 0.003 * variantScale,
            -0.18 + 0.01 * static_cast<double>(i % 4) - 0.002 * variantScale;

        Eigen::VectorXd targetVelocity(3);
        targetVelocity << 0.08 + 0.01 * static_cast<double>(i % 2)
                              + 0.002 * variantScale,
            -0.03 - 0.004 * static_cast<double>(i % 3), 0.02;

        const auto& primaryStructure
            = registry.get<sx::comps::MultibodyStructure>(robot.multibody);
        const std::vector<compute::LinkContact> linkContacts{contact};
        auto primaryProblem = compute::assembleMultibodyLinkContactProblem(
            registry, primaryStructure, nextVelocity, kTimeStep, linkContacts);
        if (primaryProblem.rows.size() != 1u) {
          errorMessage
              = "cross-multibody articulated contact did not assemble one row";
          return std::nullopt;
        }

        const auto& targetStructure
            = registry.get<sx::comps::MultibodyStructure>(target.multibody);
        const std::vector<compute::LinkContact> targetContacts;
        auto targetProblem = compute::assembleMultibodyLinkContactProblem(
            registry,
            targetStructure,
            targetVelocity,
            kTimeStep,
            targetContacts);
        if (!targetProblem.rows.empty()) {
          errorMessage
              = "cross-multibody target contact unexpectedly assembled rows";
          return std::nullopt;
        }

        multibodyContacts.push_back(
            {robot.multibody, std::move(primaryProblem)});
        multibodyVelocities.push_back(std::move(nextVelocity));
        multibodyContacts.push_back(
            {target.multibody, std::move(targetProblem)});
        multibodyVelocities.push_back(std::move(targetVelocity));
        continue;
      }
    }

    const auto& structure
        = registry.get<sx::comps::MultibodyStructure>(robot.multibody);
    const std::vector<compute::LinkContact> linkContacts{contact};
    auto linkProblem = compute::assembleMultibodyLinkContactProblem(
        registry, structure, nextVelocity, kTimeStep, linkContacts);
    if (linkProblem.rows.size() != 1u || !linkProblem.rows[0].active) {
      errorMessage = "articulated link contact did not assemble one active row";
      return std::nullopt;
    }
    const auto& row = linkProblem.rows[0];
    if (row.normalDenominator <= 0.0 || row.tangentDenominator1 <= 0.0
        || row.tangentDenominator2 <= 0.0) {
      errorMessage = "articulated link contact has a nonpositive denominator";
      return std::nullopt;
    }

    multibodyContacts.push_back({robot.multibody, std::move(linkProblem)});
    multibodyVelocities.push_back(std::move(nextVelocity));
  }

  if (!completeBenchmarkCrossMultibodyRows(
          registry, multibodyContacts, multibodyVelocities, errorMessage)) {
    return std::nullopt;
  }

  std::size_t activeRowCount = 0;
  for (const auto& multibodyContact : multibodyContacts) {
    for (const auto& row : multibodyContact.problem.rows) {
      if (!row.active) {
        errorMessage = "articulated link contact row is inactive";
        return std::nullopt;
      }
      if (row.normalDenominator <= 0.0 || row.tangentDenominator1 <= 0.0
          || row.tangentDenominator2 <= 0.0) {
        errorMessage = "articulated link contact has a nonpositive denominator";
        return std::nullopt;
      }
      ++activeRowCount;
    }
  }
  if (activeRowCount != static_cast<std::size_t>(contactCount)) {
    errorMessage = "articulated unified contact active row count changed";
    return std::nullopt;
  }

  compute::RigidBodyContactProblem emptyRigid;
  const auto unified = compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);
  const Eigen::Index expectedRows
      = compute::UnifiedConstraintProblem::kRowsPerContact * contactCount;
  if (unified.delassus.rows() != expectedRows
      || unified.delassus.cols() != expectedRows
      || unified.rhs.size() != expectedRows
      || unified.findex.size() != expectedRows) {
    errorMessage = "articulated unified contact shape changed";
    return std::nullopt;
  }

  const auto solution = compute::solveUnifiedConstraintProblem(unified);
  if (!solution.succeeded) {
    errorMessage = "articulated unified contact reference solve failed";
    return std::nullopt;
  }

  LcpProblem problem(
      unified.delassus, unified.rhs, unified.lo, unified.hi, unified.findex);
  const auto check = dart::test::CheckLcpSolution(
      problem, solution.lambda, MakeBenchmarkOptions(100));
  if (!check.ok) {
    errorMessage
        = "articulated unified contact reference violates LCP contract";
    return std::nullopt;
  }

  return WorldContactBenchmarkProblem{
      std::move(problem),
      solution.lambda,
      static_cast<std::size_t>(contactCount),
      benchmarkCase == ArticulatedContactBenchmarkCase::RigidImpact
          ? static_cast<std::size_t>(contactCount)
          : std::size_t{0}};
}

std::optional<WorldContactBenchmarkProblem>
MakeStaggeringContactPipelineSweepProblem(
    const StaggeringContactPipelineSweepCase testCase,
    std::string& errorMessage)
{
  switch (testCase.kind) {
    case StaggeringContactPipelineKind::WorldSeparated:
      return MakeWorldContactBenchmarkProblem(
          testCase.contactOrShapeCount, errorMessage);
    case StaggeringContactPipelineKind::WorldStack:
      return MakeWorldStackContactBenchmarkProblem(
          testCase.contactOrShapeCount, errorMessage);
    case StaggeringContactPipelineKind::ArticulatedGround:
      return MakeArticulatedUnifiedContactBenchmarkProblem(
          ArticulatedContactBenchmarkCase::Ground,
          testCase.contactOrShapeCount,
          errorMessage);
    case StaggeringContactPipelineKind::ArticulatedRigidImpact:
      return MakeArticulatedUnifiedContactBenchmarkProblem(
          ArticulatedContactBenchmarkCase::RigidImpact,
          testCase.contactOrShapeCount,
          errorMessage);
    case StaggeringContactPipelineKind::ArticulatedCrossLinkImpact:
      return MakeArticulatedUnifiedContactBenchmarkProblem(
          ArticulatedContactBenchmarkCase::CrossLinkImpact,
          testCase.contactOrShapeCount,
          errorMessage);
  }

  errorMessage = "unknown Staggering contact-pipeline sweep case";
  return std::nullopt;
}

std::optional<LcpProblem> MakeContactNormalStandardProblem(
    const WorldContactBenchmarkProblem& fixture, std::string& errorMessage)
{
  std::vector<Eigen::Index> normalRows;
  normalRows.reserve(fixture.contactCount);
  for (Eigen::Index row = 0; row < fixture.problem.findex.size(); ++row) {
    if (fixture.problem.findex[row] < 0) {
      normalRows.push_back(row);
    }
  }

  if (normalRows.empty()) {
    errorMessage = "contact-normal standard problem has no normal rows";
    return std::nullopt;
  }

  if (normalRows.size() != fixture.contactCount) {
    errorMessage = "contact-normal row count does not match contact count";
    return std::nullopt;
  }

  const Eigen::Index n = static_cast<Eigen::Index>(normalRows.size());
  Eigen::MatrixXd A(n, n);
  Eigen::VectorXd b(n);
  for (Eigen::Index r = 0; r < n; ++r) {
    b[r] = fixture.problem.b[normalRows[static_cast<std::size_t>(r)]];
    for (Eigen::Index c = 0; c < n; ++c) {
      A(r, c) = fixture.problem.A(
          normalRows[static_cast<std::size_t>(r)],
          normalRows[static_cast<std::size_t>(c)]);
    }
  }

  const double inf = std::numeric_limits<double>::infinity();
  return LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, inf),
      Eigen::VectorXi::Constant(n, -1));
}

std::optional<WorldContactBenchmarkBatch> MakeWorldContactBenchmarkBatch(
    std::string& errorMessage,
    WorldContactBatchKind batchKind = WorldContactBatchKind::Baseline)
{
  constexpr std::array<int, 3> kSeparatedContactCounts{1, 2, 4};
  constexpr std::array<int, 2> kBaselineStackSphereCounts{2, 3};
  constexpr std::array<int, 4> kStressStackSphereCounts{2, 3, 4, 5};
  constexpr std::array<StaggeringContactPipelineSweepCase, 5>
      kContactPipeline32Cases{{
          {StaggeringContactPipelineKind::WorldSeparated,
           32,
           "WorldSeparated32"},
          {StaggeringContactPipelineKind::WorldStack, 32, "WorldStack32"},
          {StaggeringContactPipelineKind::ArticulatedGround,
           32,
           "ArticulatedGround32"},
          {StaggeringContactPipelineKind::ArticulatedRigidImpact,
           32,
           "ArticulatedRigidImpact32"},
          {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
           32,
           "ArticulatedCrossLinkImpact32"},
      }};

  WorldContactBenchmarkBatch batch;
  if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    batch.problems.reserve(kContactPipeline32Cases.size());
  } else {
    batch.problems.reserve(
        kSeparatedContactCounts.size()
        + (batchKind == WorldContactBatchKind::StressStack
               ? kStressStackSphereCounts.size()
               : kBaselineStackSphereCounts.size()));
  }

  auto appendProblem = [&batch](WorldContactBenchmarkProblem& fixture) {
    batch.totalProblemSize += fixture.problem.b.size();
    batch.totalContactCount += fixture.contactCount;
    batch.totalBodyCount += fixture.bodyCount;
    batch.problems.push_back(std::move(fixture.problem));
  };

  if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    for (const auto testCase : kContactPipeline32Cases) {
      auto fixture
          = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
      if (!fixture.has_value()) {
        return std::nullopt;
      }
      appendProblem(*fixture);
    }
    return batch;
  }

  for (const int contactCount : kSeparatedContactCounts) {
    auto fixture = MakeWorldContactBenchmarkProblem(contactCount, errorMessage);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    appendProblem(*fixture);
  }

  const auto appendStackProblem = [&](const int sphereCount) {
    auto fixture
        = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage);
    if (!fixture.has_value()) {
      return false;
    }
    appendProblem(*fixture);
    return true;
  };

  if (batchKind == WorldContactBatchKind::StressStack) {
    for (const int sphereCount : kStressStackSphereCounts) {
      if (!appendStackProblem(sphereCount)) {
        return std::nullopt;
      }
    }
  } else {
    for (const int sphereCount : kBaselineStackSphereCounts) {
      if (!appendStackProblem(sphereCount)) {
        return std::nullopt;
      }
    }
  }

  return batch;
}

std::optional<WorldContactBenchmarkBatch> MakeWorldBoxContactBenchmarkBatch(
    int boxCount, int batchSize, std::string& errorMessage)
{
  if (boxCount <= 0) {
    errorMessage = "dense box-contact batch box count must be positive";
    return std::nullopt;
  }
  if (batchSize <= 0) {
    errorMessage = "dense box-contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  Eigen::Index expectedRows = 0;
  for (const int i : std::views::iota(0, batchSize)) {
    auto fixture
        = MakeWorldBoxContactBenchmarkProblem(errorMessage, i, boxCount);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    if (fixture->contactCount != static_cast<std::size_t>(4 * boxCount)) {
      errorMessage = "dense box-contact batch lost face contact coverage";
      return std::nullopt;
    }
    if (expectedRows == 0) {
      expectedRows = fixture->problem.b.size();
    } else if (fixture->problem.b.size() != expectedRows) {
      errorMessage = "dense box-contact batch problem shape changed";
      return std::nullopt;
    }
    if (fixture->problem.b.size() != static_cast<Eigen::Index>(12 * boxCount)) {
      errorMessage = "dense box-contact batch row count changed";
      return std::nullopt;
    }

    batch.totalProblemSize += fixture->problem.b.size();
    batch.totalContactCount += fixture->contactCount;
    batch.totalBodyCount += fixture->bodyCount;
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

  #if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
std::optional<WorldContactBenchmarkBatch>
MakeHomogeneousWorldContactBenchmarkBatch(
    int contactCount, int batchSize, std::string& errorMessage)
{
  if (batchSize <= 0) {
    errorMessage = "world-contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  for (const int i : std::views::iota(0, batchSize)) {
    auto fixture
        = MakeWorldContactBenchmarkProblem(contactCount, errorMessage, i);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    if (fixture->problem.b.size()
        != static_cast<Eigen::Index>(3 * contactCount)) {
      errorMessage = "world-contact batch problem shape changed";
      return std::nullopt;
    }

    batch.totalProblemSize += fixture->problem.b.size();
    batch.totalContactCount += fixture->contactCount;
    batch.totalBodyCount += fixture->bodyCount;
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

std::optional<WorldContactBenchmarkBatch>
MakeHomogeneousWorldBoxContactBenchmarkBatch(
    int boxCount, int batchSize, std::string& errorMessage)
{
  if (boxCount <= 0) {
    errorMessage = "dense box-contact box count must be positive";
    return std::nullopt;
  }
  if (batchSize <= 0) {
    errorMessage = "dense box-contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  Eigen::Index expectedRows = 0;
  for (const int i : std::views::iota(0, batchSize)) {
    auto fixture
        = MakeWorldBoxContactBenchmarkProblem(errorMessage, i, boxCount);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    if (fixture->contactCount != static_cast<std::size_t>(4 * boxCount)) {
      errorMessage = "dense box-contact batch lost face contact coverage";
      return std::nullopt;
    }
    if (expectedRows == 0) {
      expectedRows = fixture->problem.b.size();
    } else if (fixture->problem.b.size() != expectedRows) {
      errorMessage = "dense box-contact batch problem shape changed";
      return std::nullopt;
    }
    if (fixture->problem.b.size() != static_cast<Eigen::Index>(12 * boxCount)) {
      errorMessage = "dense box-contact batch row count changed";
      return std::nullopt;
    }

    batch.totalProblemSize += fixture->problem.b.size();
    batch.totalContactCount += fixture->contactCount;
    batch.totalBodyCount += fixture->bodyCount;
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

std::optional<WorldContactBenchmarkBatch>
MakeHomogeneousWorldStackContactBenchmarkBatch(
    int sphereCount, int batchSize, std::string& errorMessage)
{
  if (batchSize <= 0) {
    errorMessage = "world-stack contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  for (const int i : std::views::iota(0, batchSize)) {
    auto fixture
        = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage, i);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    if (fixture->problem.b.size()
        != static_cast<Eigen::Index>(3 * sphereCount)) {
      errorMessage = "world-stack contact batch problem shape changed";
      return std::nullopt;
    }

    batch.totalProblemSize += fixture->problem.b.size();
    batch.totalContactCount += fixture->contactCount;
    batch.totalBodyCount += fixture->bodyCount;
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}
  #endif

std::unique_ptr<sx::World> MakeWorldSeparatedStepBenchmarkWorld(
    int sphereCount, std::string& errorMessage)
{
  if (sphereCount <= 0) {
    errorMessage = "sphere count must be positive";
    return nullptr;
  }

  constexpr double kFriction = 0.7;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));
  ground.setFriction(kFriction);

  const int columns = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(sphereCount))));
  constexpr double kSpacing = 2.0;
  for (const int i : std::views::iota(0, sphereCount)) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.35 - 0.03 * static_cast<double>(i % 4),
        -0.22 + 0.025 * static_cast<double>(i % 5),
        -0.02);
    auto sphere
        = world->addRigidBody("sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(sphereCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " separated contacts, expected "
                   + std::to_string(sphereCount);
    return nullptr;
  }

  return world;
}

std::unique_ptr<sx::World> MakeWorldBoxStepBenchmarkWorld(
    int boxCount, std::string& errorMessage)
{
  if (boxCount <= 0) {
    errorMessage = "box count must be positive";
    return nullptr;
  }

  constexpr double kFriction = 0.5;
  sx::WorldOptions options;
  options.timeStep = boxCount >= 24 ? 0.001 : 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  const double groundHalfExtent = MakeDenseBoxGroundHalfExtent(boxCount);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(groundHalfExtent, groundHalfExtent, 0.5)));
  ground.setFriction(kFriction);

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  constexpr double kSpacing = 2.0;
  for (const int i : std::views::iota(0, boxCount)) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions boxOptions;
    boxOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    boxOptions.linearVelocity = Eigen::Vector3d(
        0.35 - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.01 * static_cast<double>(i % 2),
        -0.02);
    auto box = world->addRigidBody("box_" + std::to_string(i), boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world->collide();
  const auto expectedContacts = static_cast<std::size_t>(4 * boxCount);
  if (contacts.size() != expectedContacts) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " dense box contacts, expected "
                   + std::to_string(expectedContacts);
    return nullptr;
  }

  return world;
}

constexpr double kBilliardsRadius = 0.14;
constexpr double kBilliardsInitialSpeed = 1.0;
constexpr double kBilliardsInitialEnergy
    = 0.5 * kBilliardsInitialSpeed * kBilliardsInitialSpeed;

Eigen::Vector3d MakeBilliardsCuePosition(int index)
{
  return Eigen::Vector3d(-0.135, 0.8 * static_cast<double>(index), 0.0);
}

Eigen::Vector3d MakeBilliardsTargetPosition(int index)
{
  return Eigen::Vector3d(0.135, 0.8 * static_cast<double>(index), 0.0);
}

std::unique_ptr<sx::World> MakeWorldBilliardsStepBenchmarkWorld(
    int pairCount, std::string& errorMessage)
{
  if (pairCount <= 0) {
    errorMessage = "billiards pair count must be positive";
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.001;
  options.gravity = Eigen::Vector3d::Zero();
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  for (const int i : std::views::iota(0, pairCount)) {
    sx::RigidBodyOptions cueOptions;
    cueOptions.position = MakeBilliardsCuePosition(i);
    cueOptions.linearVelocity
        = Eigen::Vector3d(kBilliardsInitialSpeed, 0.0, 0.0);
    auto cue
        = world->addRigidBody("billiards_cue_" + std::to_string(i), cueOptions);
    cue.setCollisionShape(sx::CollisionShape::makeSphere(kBilliardsRadius));
    cue.setMass(1.0);
    cue.setFriction(0.0);
    cue.setRestitution(1.0);

    sx::RigidBodyOptions targetOptions;
    targetOptions.position = MakeBilliardsTargetPosition(i);
    auto target = world->addRigidBody(
        "billiards_target_" + std::to_string(i), targetOptions);
    target.setCollisionShape(sx::CollisionShape::makeSphere(kBilliardsRadius));
    target.setMass(1.0);
    target.setFriction(0.0);
    target.setRestitution(1.0);
  }

  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(pairCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " billiards contacts, expected "
                   + std::to_string(pairCount);
    return nullptr;
  }

  return world;
}

Eigen::Vector3d MakeCardPileHalfExtents()
{
  return Eigen::Vector3d(0.30, 0.18, 0.012);
}

Eigen::Vector2d MakeCardPileInitialOffset(int index)
{
  static constexpr std::array<double, 7> kOffsetX{
      0.000, 0.018, -0.014, 0.010, -0.020, 0.012, -0.006};
  static constexpr std::array<double, 7> kOffsetY{
      0.000, -0.010, 0.014, 0.020, -0.016, 0.006, -0.020};
  const auto wrappedIndex
      = static_cast<std::size_t>(index % static_cast<int>(kOffsetX.size()));
  const double repeatOffset
      = 0.002 * static_cast<double>(index / static_cast<int>(kOffsetX.size()));
  return Eigen::Vector2d(
      kOffsetX[wrappedIndex] + repeatOffset, kOffsetY[wrappedIndex]);
}

double MakeCardPileInitialYaw(int index)
{
  static constexpr std::array<double, 7> kYaws{
      -0.08, 0.04, 0.10, -0.05, 0.07, -0.11, 0.03};
  return kYaws[static_cast<std::size_t>(
      index % static_cast<int>(kYaws.size()))];
}

Eigen::Vector3d MakeCardPileInitialPosition(int index)
{
  const Eigen::Vector3d halfExtents = MakeCardPileHalfExtents();
  const Eigen::Vector2d offset = MakeCardPileInitialOffset(index);
  return Eigen::Vector3d(
      offset.x(),
      offset.y(),
      halfExtents.z() + static_cast<double>(index) * 2.0 * halfExtents.z());
}

std::unique_ptr<sx::World> MakeWorldCardPileStepBenchmarkWorld(
    int cardCount, std::string& errorMessage)
{
  if (cardCount < 2) {
    errorMessage = "card count must be at least two for a contact pile";
    return nullptr;
  }

  constexpr double kFriction = 0.9;
  sx::WorldOptions options;
  options.timeStep = cardCount >= 8 ? 0.001 : 0.002;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto ground = world->addRigidBody("card_pile_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.05)));
  ground.setFriction(kFriction);

  const Eigen::Vector3d halfExtents = MakeCardPileHalfExtents();
  for (const int i : std::views::iota(0, cardCount)) {
    sx::RigidBodyOptions cardOptions;
    cardOptions.position = MakeCardPileInitialPosition(i);
    cardOptions.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(MakeCardPileInitialYaw(i), Eigen::Vector3d::UnitZ()));
    cardOptions.linearVelocity = i == cardCount - 1
                                     ? Eigen::Vector3d(0.16, -0.03, -0.02)
                                     : Eigen::Vector3d(0.0, 0.0, -0.01);
    auto card = world->addRigidBody("card_" + std::to_string(i), cardOptions);
    card.setCollisionShape(sx::CollisionShape::makeBox(halfExtents));
    card.setFriction(kFriction);
    card.setMass(0.08);
    card.setInertia(Eigen::Vector3d(0.003, 0.006, 0.008).asDiagonal());
  }

  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() < static_cast<std::size_t>(cardCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " card-pile contacts, expected at least "
                   + std::to_string(cardCount);
    return nullptr;
  }

  return world;
}

struct WorldSeparatedStepCheck
{
  bool ok{true};
  double maxHeightError{0.0};
  double maxVerticalSpeed{0.0};
  double maxFinalTangentialSpeed{0.0};
  double minTangentialSpeedDrop{std::numeric_limits<double>::infinity()};
};

WorldSeparatedStepCheck CheckWorldSeparatedStepInvariants(
    sx::World& world, int sphereCount)
{
  WorldSeparatedStepCheck check;
  for (const int i : std::views::iota(0, sphereCount)) {
    auto sphere = world.getRigidBody("sphere_" + std::to_string(i));
    if (!sphere.has_value()) {
      check.ok = false;
      return check;
    }

    const Eigen::Vector2d initialTangentialVelocity(
        0.35 - 0.03 * static_cast<double>(i % 4),
        -0.22 + 0.025 * static_cast<double>(i % 5));
    const double initialTangentialSpeed = initialTangentialVelocity.norm();
    const double finalTangentialSpeed
        = sphere->getLinearVelocity().head<2>().norm();
    const double heightError = std::abs(sphere->getTranslation().z() - 0.5);
    check.maxHeightError = std::max(check.maxHeightError, heightError);
    check.maxVerticalSpeed = std::max(
        check.maxVerticalSpeed, std::abs(sphere->getLinearVelocity().z()));
    check.maxFinalTangentialSpeed
        = std::max(check.maxFinalTangentialSpeed, finalTangentialSpeed);
    check.minTangentialSpeedDrop = std::min(
        check.minTangentialSpeedDrop,
        initialTangentialSpeed - finalTangentialSpeed);
    check.ok = check.ok && sphere->getTranslation().allFinite()
               && sphere->getLinearVelocity().allFinite()
               && sphere->getTranslation().z() >= 0.5 - 1e-3
               && heightError <= 2e-2
               && std::abs(sphere->getLinearVelocity().z()) < 0.1
               && finalTangentialSpeed < initialTangentialSpeed;
  }

  return check;
}

struct WorldBoxStepCheck
{
  bool ok{true};
  double maxHeightError{0.0};
  double maxVerticalSpeed{0.0};
  double maxFinalTangentialSpeed{0.0};
  double minTangentialSpeedDrop{std::numeric_limits<double>::infinity()};
};

WorldBoxStepCheck CheckWorldBoxStepInvariants(sx::World& world, int boxCount)
{
  WorldBoxStepCheck check;
  for (const int i : std::views::iota(0, boxCount)) {
    auto box = world.getRigidBody("box_" + std::to_string(i));
    if (!box.has_value()) {
      check.ok = false;
      return check;
    }

    const Eigen::Vector2d initialTangentialVelocity(
        0.35 - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.01 * static_cast<double>(i % 2));
    const double initialTangentialSpeed = initialTangentialVelocity.norm();
    const double finalTangentialSpeed
        = box->getLinearVelocity().head<2>().norm();
    const double heightError = std::abs(box->getTranslation().z() - 0.5);

    check.maxHeightError = std::max(check.maxHeightError, heightError);
    check.maxVerticalSpeed = std::max(
        check.maxVerticalSpeed, std::abs(box->getLinearVelocity().z()));
    check.maxFinalTangentialSpeed
        = std::max(check.maxFinalTangentialSpeed, finalTangentialSpeed);
    check.minTangentialSpeedDrop = std::min(
        check.minTangentialSpeedDrop,
        initialTangentialSpeed - finalTangentialSpeed);
    check.ok = check.ok && box->getTranslation().allFinite()
               && box->getLinearVelocity().allFinite()
               && box->getTranslation().z() >= 0.5 - 1e-3 && heightError <= 2e-2
               && std::abs(box->getLinearVelocity().z()) < 0.1
               && finalTangentialSpeed < initialTangentialSpeed;
  }

  return check;
}

struct WorldBilliardsStepCheck
{
  bool ok{true};
  double maxMomentumError{0.0};
  double maxEnergyError{0.0};
  double minTargetSpeed{std::numeric_limits<double>::infinity()};
  double maxTargetSpeed{0.0};
  double targetSpeedSpread{0.0};
  double maxCueSpeed{0.0};
  double maxOffAxisSpeed{0.0};
  std::size_t contactCount{0};
};

WorldBilliardsStepCheck CheckWorldBilliardsStepInvariants(
    sx::World& world, int pairCount)
{
  WorldBilliardsStepCheck check;
  for (const int i : std::views::iota(0, pairCount)) {
    auto cue = world.getRigidBody("billiards_cue_" + std::to_string(i));
    auto target = world.getRigidBody("billiards_target_" + std::to_string(i));
    if (!cue.has_value() || !target.has_value()) {
      check.ok = false;
      return check;
    }

    const Eigen::Vector3d cueVelocity = cue->getLinearVelocity();
    const Eigen::Vector3d targetVelocity = target->getLinearVelocity();
    const double momentumX = cueVelocity.x() + targetVelocity.x();
    const double energy
        = 0.5 * (cueVelocity.squaredNorm() + targetVelocity.squaredNorm());
    check.maxMomentumError = std::max(
        check.maxMomentumError, std::abs(momentumX - kBilliardsInitialSpeed));
    check.maxEnergyError = std::max(
        check.maxEnergyError, std::abs(energy - kBilliardsInitialEnergy));
    const double targetSpeed = targetVelocity.norm();
    check.minTargetSpeed = std::min(check.minTargetSpeed, targetSpeed);
    check.maxTargetSpeed = std::max(check.maxTargetSpeed, targetSpeed);
    check.maxCueSpeed = std::max(check.maxCueSpeed, cueVelocity.norm());
    check.maxOffAxisSpeed = std::max(
        check.maxOffAxisSpeed,
        std::max(
            std::max(std::abs(cueVelocity.y()), std::abs(targetVelocity.y())),
            std::max(std::abs(cueVelocity.z()), std::abs(targetVelocity.z()))));
    check.ok = check.ok && cue->getTranslation().allFinite()
               && target->getTranslation().allFinite()
               && cueVelocity.allFinite() && targetVelocity.allFinite()
               && check.maxMomentumError < 1e-8 && check.maxEnergyError < 1e-8
               && targetVelocity.x() > 0.5 * kBilliardsInitialSpeed
               && cueVelocity.x() < 0.5 * kBilliardsInitialSpeed
               && std::abs(cueVelocity.y()) < 1e-10
               && std::abs(targetVelocity.y()) < 1e-10
               && std::abs(cueVelocity.z()) < 1e-10
               && std::abs(targetVelocity.z()) < 1e-10;
  }

  check.targetSpeedSpread = check.maxTargetSpeed - check.minTargetSpeed;
  check.ok = check.ok && check.targetSpeedSpread < 1e-10
             && check.maxOffAxisSpeed < 1e-10;
  check.contactCount = world.collide().size();
  return check;
}

struct WorldCardPileStepCheck
{
  bool ok{true};
  double maxSpread{0.0};
  double maxHeightLoss{0.0};
  double maxVerticalSpeed{0.0};
  double maxAngularSpeed{0.0};
  std::size_t contactCount{0};
};

WorldCardPileStepCheck CheckWorldCardPileStepInvariants(
    sx::World& world, int cardCount)
{
  WorldCardPileStepCheck check;
  const double initialTopZ = MakeCardPileInitialPosition(cardCount - 1).z();
  double currentTopZ = -std::numeric_limits<double>::infinity();

  for (const int i : std::views::iota(0, cardCount)) {
    auto card = world.getRigidBody("card_" + std::to_string(i));
    if (!card.has_value()) {
      check.ok = false;
      return check;
    }

    const Eigen::Vector3d initialPosition = MakeCardPileInitialPosition(i);
    const Eigen::Vector3d position = card->getTranslation();
    const Eigen::Vector3d linearVelocity = card->getLinearVelocity();
    const Eigen::Vector3d angularVelocity = card->getAngularVelocity();
    const double spread
        = (position.head<2>() - initialPosition.head<2>()).norm();
    check.maxSpread = std::max(check.maxSpread, spread);
    check.maxVerticalSpeed
        = std::max(check.maxVerticalSpeed, std::abs(linearVelocity.z()));
    check.maxAngularSpeed
        = std::max(check.maxAngularSpeed, angularVelocity.norm());
    currentTopZ = std::max(currentTopZ, position.z());
    check.ok = check.ok && position.allFinite() && linearVelocity.allFinite()
               && angularVelocity.allFinite() && position.z() > -0.05
               && spread < 1.0 && std::abs(linearVelocity.z()) < 3.0
               && angularVelocity.norm() < 40.0;
  }

  check.maxHeightLoss = std::max(0.0, initialTopZ - currentTopZ);
  check.contactCount = world.collide().size();
  check.ok = check.ok && check.maxHeightLoss < 0.5
             && check.contactCount >= static_cast<std::size_t>(cardCount);
  return check;
}

std::unique_ptr<sx::World> MakeWorldArticulatedGroundStepBenchmarkWorld(
    int linkCount, std::string& errorMessage)
{
  if (linkCount <= 0) {
    errorMessage = "articulated link count must be positive";
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.002;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.5;
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  const double groundHalfExtent
      = MakePositiveGridGroundHalfExtent(linkCount, kSpacing, 24.0);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(groundHalfExtent, groundHalfExtent, 0.5)));

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(linkCount))));
  for (const int i : std::views::iota(0, linkCount)) {
    const int row = i / columns;
    const int col = i - row * columns;

    auto robot = world->addMultibody("leg_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    spec.transformFromParent = Eigen::Isometry3d::Identity();
    spec.transformFromParent.translation() = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.0);

    auto leg = robot.addLink("leg", base, spec);
    leg.setMass(1.0);
    leg.setInertia(Eigen::Matrix3d::Identity());
    leg.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = leg.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, -0.305));
    joint.setVelocity(Eigen::VectorXd::Constant(1, -0.05));
  }

  world->enterSimulationMode();
  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(linkCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " articulated contacts, expected "
                   + std::to_string(linkCount);
    return nullptr;
  }

  return world;
}

struct WorldArticulatedGroundStepCheck
{
  bool ok{true};
  double maxHeightError{0.0};
  double maxAbsJointVelocity{0.0};
};

WorldArticulatedGroundStepCheck CheckWorldArticulatedGroundStepInvariants(
    sx::World& world, int linkCount)
{
  WorldArticulatedGroundStepCheck check;
  for (const int i : std::views::iota(0, linkCount)) {
    auto robot = world.getMultibody("leg_robot_" + std::to_string(i));
    if (!robot.has_value()) {
      check.ok = false;
      return check;
    }
    auto leg = robot->getLink("leg");
    if (!leg.has_value()) {
      check.ok = false;
      return check;
    }

    const auto joint = leg->getParentJoint();
    const double linkZ = leg->getWorldTransform().translation().z();
    const double jointVelocity = joint.getVelocity()[0];
    const double heightError = std::abs(linkZ + 0.3);
    check.maxHeightError = std::max(check.maxHeightError, heightError);
    check.maxAbsJointVelocity
        = std::max(check.maxAbsJointVelocity, std::abs(jointVelocity));
    check.ok = check.ok && std::isfinite(linkZ) && std::isfinite(jointVelocity)
               && linkZ >= -0.315 && heightError <= 2e-2
               && std::abs(jointVelocity) < 0.12;
  }

  return check;
}

std::unique_ptr<sx::World> MakeWorldArticulatedRigidImpactStepBenchmarkWorld(
    int pairCount, std::string& errorMessage)
{
  if (pairCount <= 0) {
    errorMessage = "articulated impact pair count must be positive";
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.001;
  options.gravity = Eigen::Vector3d::Zero();
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.0;
  for (const int i : std::views::iota(0, pairCount)) {
    const double y = kSpacing * static_cast<double>(i);
    auto robot = world->addMultibody("striker_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "rail";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitX();
    spec.transformFromParent = Eigen::Isometry3d::Identity();
    spec.transformFromParent.translation() = Eigen::Vector3d(0.0, y, 0.0);

    auto striker = robot.addLink("striker", base, spec);
    striker.setMass(2.0);
    striker.setInertia(Eigen::Matrix3d::Identity());
    striker.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    striker.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 1.0));

    sx::RigidBodyOptions targetOptions;
    targetOptions.position = Eigen::Vector3d(0.399, y, 0.0);
    targetOptions.mass = 1.0;
    targetOptions.inertia = Eigen::Matrix3d::Identity();
    auto target
        = world->addRigidBody("target_" + std::to_string(i), targetOptions);
    target.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }

  world->enterSimulationMode();
  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(pairCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " articulated impact contacts, expected "
                   + std::to_string(pairCount);
    return nullptr;
  }

  return world;
}

struct WorldArticulatedRigidImpactStepCheck
{
  bool ok{true};
  double maxMomentumError{0.0};
  double minTargetVelocity{std::numeric_limits<double>::infinity()};
  double maxStrikerVelocity{-std::numeric_limits<double>::infinity()};
};

WorldArticulatedRigidImpactStepCheck
CheckWorldArticulatedRigidImpactStepInvariants(sx::World& world, int pairCount)
{
  WorldArticulatedRigidImpactStepCheck check;
  for (const int i : std::views::iota(0, pairCount)) {
    auto robot = world.getMultibody("striker_robot_" + std::to_string(i));
    auto target = world.getRigidBody("target_" + std::to_string(i));
    if (!robot.has_value() || !target.has_value()) {
      check.ok = false;
      return check;
    }
    auto striker = robot->getLink("striker");
    if (!striker.has_value()) {
      check.ok = false;
      return check;
    }

    const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
    const double targetVelocity = target->getLinearVelocity().x();
    const double momentum = 2.0 * strikerVelocity + targetVelocity;
    check.maxMomentumError
        = std::max(check.maxMomentumError, std::abs(momentum - 2.0));
    check.minTargetVelocity = std::min(check.minTargetVelocity, targetVelocity);
    check.maxStrikerVelocity
        = std::max(check.maxStrikerVelocity, strikerVelocity);
    check.ok = check.ok && std::isfinite(strikerVelocity)
               && std::isfinite(targetVelocity) && strikerVelocity < 1.0
               && targetVelocity > 0.0 && std::abs(momentum - 2.0) < 1e-9;
  }

  return check;
}

std::unique_ptr<sx::World> MakeWorldArticulatedLinkImpactStepBenchmarkWorld(
    int pairCount, std::string& errorMessage)
{
  if (pairCount <= 0) {
    errorMessage = "articulated link-impact pair count must be positive";
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.001;
  options.gravity = Eigen::Vector3d::Zero();
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.0;
  for (const int i : std::views::iota(0, pairCount)) {
    const double y = kSpacing * static_cast<double>(i);

    auto strikerRobot
        = world->addMultibody("striker_robot_" + std::to_string(i));
    auto strikerBase = strikerRobot.addLink("base");
    sx::JointSpec strikerSpec;
    strikerSpec.name = "rail";
    strikerSpec.type = sx::JointType::Prismatic;
    strikerSpec.axis = Eigen::Vector3d::UnitX();
    strikerSpec.transformFromParent = Eigen::Isometry3d::Identity();
    strikerSpec.transformFromParent.translation()
        = Eigen::Vector3d(0.0, y, 0.0);

    auto striker = strikerRobot.addLink("striker", strikerBase, strikerSpec);
    striker.setMass(2.0);
    striker.setInertia(Eigen::Matrix3d::Identity());
    striker.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    striker.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 1.0));

    auto targetRobot = world->addMultibody("target_robot_" + std::to_string(i));
    auto targetBase = targetRobot.addLink("base");
    sx::JointSpec targetSpec;
    targetSpec.name = "rail";
    targetSpec.type = sx::JointType::Prismatic;
    targetSpec.axis = Eigen::Vector3d::UnitX();
    targetSpec.transformFromParent = Eigen::Isometry3d::Identity();
    targetSpec.transformFromParent.translation()
        = Eigen::Vector3d(0.399, y, 0.0);

    auto target = targetRobot.addLink("target", targetBase, targetSpec);
    target.setMass(1.0);
    target.setInertia(Eigen::Matrix3d::Identity());
    target.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }

  world->enterSimulationMode();
  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(pairCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " articulated link-impact contacts, expected "
                   + std::to_string(pairCount);
    return nullptr;
  }

  return world;
}

struct WorldArticulatedLinkImpactStepCheck
{
  bool ok{true};
  double maxMomentumError{0.0};
  double minTargetVelocity{std::numeric_limits<double>::infinity()};
  double maxStrikerVelocity{-std::numeric_limits<double>::infinity()};
  double minRelativeVelocity{std::numeric_limits<double>::infinity()};
};

WorldArticulatedLinkImpactStepCheck
CheckWorldArticulatedLinkImpactStepInvariants(sx::World& world, int pairCount)
{
  WorldArticulatedLinkImpactStepCheck check;
  for (const int i : std::views::iota(0, pairCount)) {
    auto strikerRobot
        = world.getMultibody("striker_robot_" + std::to_string(i));
    auto targetRobot = world.getMultibody("target_robot_" + std::to_string(i));
    if (!strikerRobot.has_value() || !targetRobot.has_value()) {
      check.ok = false;
      return check;
    }
    auto striker = strikerRobot->getLink("striker");
    auto target = targetRobot->getLink("target");
    if (!striker.has_value() || !target.has_value()) {
      check.ok = false;
      return check;
    }

    const double strikerVelocity = striker->getParentJoint().getVelocity()[0];
    const double targetVelocity = target->getParentJoint().getVelocity()[0];
    const double relativeVelocity = targetVelocity - strikerVelocity;
    const double momentum = 2.0 * strikerVelocity + targetVelocity;
    check.maxMomentumError
        = std::max(check.maxMomentumError, std::abs(momentum - 2.0));
    check.minTargetVelocity = std::min(check.minTargetVelocity, targetVelocity);
    check.maxStrikerVelocity
        = std::max(check.maxStrikerVelocity, strikerVelocity);
    check.minRelativeVelocity
        = std::min(check.minRelativeVelocity, relativeVelocity);
    check.ok = check.ok && std::isfinite(strikerVelocity)
               && std::isfinite(targetVelocity) && strikerVelocity < 1.0
               && targetVelocity > 0.0 && relativeVelocity >= -1e-9
               && std::abs(momentum - 2.0) < 1e-9;
  }

  return check;
}

std::unique_ptr<sx::World>
MakeWorldCartesianArticulatedGroundStepBenchmarkWorld(
    int chainCount, std::string& errorMessage)
{
  if (chainCount <= 0) {
    errorMessage = "cartesian articulated chain count must be positive";
    return nullptr;
  }

  sx::WorldOptions options;
  options.timeStep = 0.002;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  constexpr double kSpacing = 1.5;
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  const double groundHalfExtent
      = MakePositiveGridGroundHalfExtent(chainCount, kSpacing, 24.0);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(groundHalfExtent, groundHalfExtent, 0.5)));

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(chainCount))));
  for (const int i : std::views::iota(0, chainCount)) {
    const int row = i / columns;
    const int col = i - row * columns;

    auto robot = world->addMultibody("cartesian_robot_" + std::to_string(i));
    auto base = robot.addLink("base");

    sx::JointSpec xSpec;
    xSpec.name = "x_slider";
    xSpec.type = sx::JointType::Prismatic;
    xSpec.axis = Eigen::Vector3d::UnitX();
    xSpec.transformFromParent = Eigen::Isometry3d::Identity();
    xSpec.transformFromParent.translation() = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.0);
    auto xLink = robot.addLink("x_link", base, xSpec);
    xLink.setMass(0.25);
    xLink.setInertia(0.05 * Eigen::Matrix3d::Identity());
    xLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.10));
    xLink.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 0.04));

    sx::JointSpec ySpec;
    ySpec.name = "y_slider";
    ySpec.type = sx::JointType::Prismatic;
    ySpec.axis = Eigen::Vector3d::UnitY();
    auto yLink = robot.addLink("y_link", xLink, ySpec);
    yLink.setMass(0.25);
    yLink.setInertia(0.05 * Eigen::Matrix3d::Identity());
    yLink.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.10));
    yLink.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -0.03));

    sx::JointSpec zSpec;
    zSpec.name = "z_slider";
    zSpec.type = sx::JointType::Prismatic;
    zSpec.axis = Eigen::Vector3d::UnitZ();
    auto tip = robot.addLink("tip", yLink, zSpec);
    tip.setMass(1.0);
    tip.setInertia(Eigen::Matrix3d::Identity());
    tip.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    tip.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.305));
    tip.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, -0.05));
  }

  world->enterSimulationMode();
  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(chainCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " cartesian articulated contacts, expected "
                   + std::to_string(chainCount);
    return nullptr;
  }

  return world;
}

struct WorldCartesianArticulatedGroundStepCheck
{
  bool ok{true};
  double maxHeightError{0.0};
  double maxAbsJointVelocity{0.0};
  double maxPlanarJointSpeed{0.0};
};

WorldCartesianArticulatedGroundStepCheck
CheckWorldCartesianArticulatedGroundStepInvariants(
    sx::World& world, int chainCount)
{
  WorldCartesianArticulatedGroundStepCheck check;
  for (const int i : std::views::iota(0, chainCount)) {
    auto robot = world.getMultibody("cartesian_robot_" + std::to_string(i));
    if (!robot.has_value()) {
      check.ok = false;
      return check;
    }
    auto xLink = robot->getLink("x_link");
    auto yLink = robot->getLink("y_link");
    auto tip = robot->getLink("tip");
    if (!xLink.has_value() || !yLink.has_value() || !tip.has_value()) {
      check.ok = false;
      return check;
    }

    const double xVelocity = xLink->getParentJoint().getVelocity()[0];
    const double yVelocity = yLink->getParentJoint().getVelocity()[0];
    const double zVelocity = tip->getParentJoint().getVelocity()[0];
    const double tipZ = tip->getWorldTransform().translation().z();
    const double heightError = std::abs(tipZ + 0.3);
    const double planarSpeed = std::hypot(xVelocity, yVelocity);
    check.maxHeightError = std::max(check.maxHeightError, heightError);
    check.maxAbsJointVelocity = std::max(
        check.maxAbsJointVelocity,
        std::max(
            {std::abs(xVelocity), std::abs(yVelocity), std::abs(zVelocity)}));
    check.maxPlanarJointSpeed
        = std::max(check.maxPlanarJointSpeed, planarSpeed);
    check.ok = check.ok && std::isfinite(xVelocity) && std::isfinite(yVelocity)
               && std::isfinite(zVelocity) && std::isfinite(tipZ)
               && tipZ >= -0.315 && heightError <= 2e-2
               && check.maxAbsJointVelocity < 0.12 && planarSpeed < 0.08;
  }

  return check;
}

std::unique_ptr<sx::World> MakeWorldStackStepBenchmarkWorld(
    int sphereCount, std::string& errorMessage)
{
  if (sphereCount < 2) {
    errorMessage = "sphere count must be at least two for a contact stack";
    return nullptr;
  }

  constexpr double kFriction = 0.6;
  const double timeStep = sphereCount >= 4 ? 0.001 : 0.005;
  sx::WorldOptions options;
  options.timeStep = timeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(8.0, 8.0, 0.5)));
  ground.setFriction(kFriction);

  for (const int i : std::views::iota(0, sphereCount)) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position
        = Eigen::Vector3d(0.0, 0.0, 0.5 + static_cast<double>(i));
    sphereOptions.linearVelocity
        = Eigen::Vector3d(0.0, 0.0, -0.02 - 0.02 * static_cast<double>(i));
    auto sphere = world->addRigidBody(
        "stack_sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world->collide();
  if (contacts.size() != static_cast<std::size_t>(sphereCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " stack contacts, expected " + std::to_string(sphereCount);
    return nullptr;
  }

  return world;
}

struct WorldStackStepCheck
{
  bool ok{true};
  double minSpacing{std::numeric_limits<double>::infinity()};
  double maxVerticalSpeed{0.0};
  double maxLateralPosition{0.0};
  double maxLateralSpeed{0.0};
};

WorldStackStepCheck CheckWorldStackStepInvariants(
    sx::World& world, int sphereCount)
{
  WorldStackStepCheck check;
  std::vector<sx::RigidBody> spheres;
  spheres.reserve(static_cast<std::size_t>(sphereCount));

  for (const int i : std::views::iota(0, sphereCount)) {
    auto sphere = world.getRigidBody("stack_sphere_" + std::to_string(i));
    if (!sphere.has_value()) {
      check.ok = false;
      return check;
    }
    spheres.push_back(*sphere);
  }

  for (const int i : std::views::iota(0, sphereCount)) {
    const sx::RigidBody& sphere = spheres[static_cast<std::size_t>(i)];
    check.maxVerticalSpeed = std::max(
        check.maxVerticalSpeed, std::abs(sphere.getLinearVelocity().z()));
    check.maxLateralPosition = std::max(
        check.maxLateralPosition, sphere.getTranslation().head<2>().norm());
    check.maxLateralSpeed = std::max(
        check.maxLateralSpeed, sphere.getLinearVelocity().head<2>().norm());

    const double expectedZ = 0.5 + static_cast<double>(i);
    check.ok = check.ok && sphere.getTranslation().allFinite()
               && sphere.getLinearVelocity().allFinite()
               && sphere.getTranslation().z() >= expectedZ - 2e-2
               && std::abs(sphere.getTranslation().z() - expectedZ) <= 6e-2
               && std::abs(sphere.getLinearVelocity().z()) < 0.15
               && sphere.getTranslation().head<2>().norm() < 1e-3
               && sphere.getLinearVelocity().head<2>().norm() < 1e-3;
  }

  for (const int i : std::views::iota(1, sphereCount)) {
    const double spacing
        = spheres[static_cast<std::size_t>(i)].getTranslation().z()
          - spheres[static_cast<std::size_t>(i - 1)].getTranslation().z();
    check.minSpacing = std::min(check.minSpacing, spacing);
    check.ok
        = check.ok && spacing >= 1.0 - 4e-2 && std::abs(spacing - 1.0) <= 8e-2;
  }

  return check;
}
#endif

std::string MakeLabel(const std::string& solver, const std::string& category)
{
  std::ostringstream out;
  out << solver << "/" << category;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
bool SupportsDenseWorldBoxContactPatch(std::string_view solverName)
{
  return solverName == "Pgs" || solverName == "RedBlackGaussSeidel"
         || solverName == "NNCG" || solverName == "Apgd" || solverName == "Tgs"
         || solverName == "Admm";
}
#endif

void AddShockPropagationCounters(
    benchmark::State& state,
    const dart::math::ShockPropagationSolver::Parameters& params)
{
  const auto blockCount = std::ssize(params.blockSizes);
  auto layerCount = std::ssize(params.layers);
  if (layerCount == 0 && blockCount > 0) {
    layerCount = 1;
  }

  int maxBlockSize = 0;
  for (const int size : params.blockSizes) {
    maxBlockSize = std::max(maxBlockSize, size);
  }

  auto maxBlocksPerLayer = std::ptrdiff_t{0};
  if (!params.layers.empty()) {
    for (const auto& layer : params.layers) {
      maxBlocksPerLayer = std::max(maxBlocksPerLayer, std::ssize(layer));
    }
  } else {
    maxBlocksPerLayer = blockCount;
  }

  state.counters["layer_count"] = layerCount;
  state.counters["block_count"] = blockCount;
  state.counters["max_block_size"] = maxBlockSize;
  state.counters["max_blocks_per_layer"] = maxBlocksPerLayer;
}

void AddShockPropagationOptionCounters(
    benchmark::State& state, const LcpOptions& options)
{
  state.counters["shock_propagation_max_iterations"] = options.maxIterations;
}

void AddBoxedSsnCounters(
    benchmark::State& state,
    const dart::math::BoxedSemiSmoothNewtonSolver::Parameters& params)
{
  state.counters["boxed_ssn_max_line_search_steps"] = params.maxLineSearchSteps;
  state.counters["boxed_ssn_step_reduction"] = params.stepReduction;
  state.counters["boxed_ssn_sufficient_decrease"] = params.sufficientDecrease;
  state.counters["boxed_ssn_min_step"] = params.minStep;
  state.counters["boxed_ssn_jacobian_regularization"]
      = params.jacobianRegularization;
  state.counters["boxed_ssn_pgs_warm_start_iterations"]
      = params.maxPgsWarmStartIterations;
  state.counters["boxed_ssn_pgs_warm_start_relaxation"]
      = params.pgsWarmStartRelaxation;
  state.counters["boxed_ssn_friction_index_exact_solve_dimension"]
      = params.maxFrictionIndexExactSolveDimension;
}

void AddSapCounters(
    benchmark::State& state, const dart::math::SapSolver::Parameters& params)
{
  state.counters["sap_regularization"] = params.regularization;
  state.counters["sap_max_line_search_iterations"]
      = params.maxLineSearchIterations;
}

void AddNncgCounters(
    benchmark::State& state, const dart::math::NncgSolver::Parameters& params)
{
  state.counters["nncg_pgs_iterations"] = params.pgsIterations;
  state.counters["nncg_restart_interval"] = params.restartInterval;
  state.counters["nncg_restart_threshold"] = params.restartThreshold;
}

void AddSymmetricPsorCounters(
    benchmark::State& state, const LcpOptions& options)
{
  state.counters["symmetric_psor_max_iterations"] = options.maxIterations;
  state.counters["symmetric_psor_relaxation"] = options.relaxation;
}

void AddBgsCounters(benchmark::State& state, const LcpOptions& options)
{
  state.counters["bgs_max_iterations"] = options.maxIterations;
}

void AddPgsCounters(benchmark::State& state, const LcpOptions& options)
{
  state.counters["pgs_max_iterations"] = options.maxIterations;
  state.counters["pgs_relaxation"] = options.relaxation;
}

void AddJacobiCounters(benchmark::State& state, const LcpOptions& options)
{
  state.counters["jacobi_max_iterations"] = options.maxIterations;
  state.counters["jacobi_relaxation"] = options.relaxation;
}

void AddBlockedJacobiCounters(
    benchmark::State& state, const LcpOptions& options)
{
  state.counters["blocked_jacobi_max_iterations"] = options.maxIterations;
  state.counters["blocked_jacobi_relaxation"] = options.relaxation;
}

void AddRedBlackGaussSeidelCounters(
    benchmark::State& state, const LcpOptions& options)
{
  state.counters["red_black_gauss_seidel_max_iterations"]
      = options.maxIterations;
  state.counters["red_black_gauss_seidel_relaxation"] = options.relaxation;
}

void AddTgsCounters(benchmark::State& state, const LcpOptions& options)
{
  state.counters["tgs_max_iterations"] = options.maxIterations;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void AddFindexShockPropagationCounters(
    benchmark::State& state, std::size_t contactCount, Eigen::Index problemSize)
{
  state.counters["layer_count"] = contactCount > 0 ? 1.0 : 0.0;
  state.counters["block_count"] = static_cast<double>(contactCount);
  state.counters["max_block_size"]
      = contactCount > 0 ? static_cast<double>(problemSize / contactCount)
                         : 0.0;
  state.counters["max_blocks_per_layer"] = static_cast<double>(contactCount);
}
#endif

struct ValidationCounters
{
  double residual{0.0};
  double complementarity{0.0};
  double boundViolation{0.0};
};

double ComplementarityViolation(
    double xi, double wi, double lo, double hi, double tol)
{
  double violation = 0.0;
  if (std::isfinite(lo) && xi < lo - tol) {
    violation = std::max(violation, lo - xi);
  }
  if (std::isfinite(hi) && xi > hi + tol) {
    violation = std::max(violation, xi - hi);
  }

  const bool hasLo = std::isfinite(lo);
  const bool hasUpper = std::isfinite(hi);
  const bool atLo = hasLo && std::abs(xi - lo) <= tol;
  const bool atHi = hasUpper && std::abs(xi - hi) <= tol;
  const bool fixed = atLo && atHi;
  const bool interior = (!atLo && !atHi) && (!hasLo || xi > lo + tol)
                        && (!hasUpper || xi < hi - tol);

  if (fixed) {
    violation = std::max(violation, std::abs(wi));
  } else if (atLo) {
    violation = std::max(violation, std::max(0.0, -wi));
  } else if (atHi) {
    violation = std::max(violation, std::max(0.0, wi));
  } else if (interior) {
    violation = std::max(violation, std::abs(wi));
  }

  return violation;
}

ValidationCounters ComputeValidationRange(
    const LcpProblem& problem,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& loEff,
    const Eigen::VectorXd& hiEff,
    double tol,
    Eigen::Index begin,
    Eigen::Index end)
{
  ValidationCounters counters;

  for (Eigen::Index i = begin; i < end; ++i) {
    const double xi = x[i];
    const double wi = problem.A.row(i).dot(x) - problem.b[i];
    const double projected
        = dart::math::detail::projectToBounds(xi - wi, loEff[i], hiEff[i]);
    counters.residual = std::max(counters.residual, std::abs(xi - projected));
    counters.complementarity = std::max(
        counters.complementarity,
        ComplementarityViolation(xi, wi, loEff[i], hiEff[i], tol));

    if (std::isfinite(loEff[i])) {
      counters.boundViolation
          = std::max(counters.boundViolation, loEff[i] - xi);
    }
    if (std::isfinite(hiEff[i])) {
      counters.boundViolation
          = std::max(counters.boundViolation, xi - hiEff[i]);
    }
  }

  counters.boundViolation = std::max(0.0, counters.boundViolation);
  return counters;
}

void MergeValidationCounters(
    ValidationCounters& target, const ValidationCounters& source)
{
  target.residual = std::max(target.residual, source.residual);
  target.complementarity
      = std::max(target.complementarity, source.complementarity);
  target.boundViolation
      = std::max(target.boundViolation, source.boundViolation);
}

void AddBackendBuildCounters(benchmark::State& state)
{
  state.counters["build_simd_enabled"]
      = DART_BM_LCP_COMPARE_DART_ENABLE_SIMD ? 1.0 : 0.0;
  state.counters["build_simd_force_scalar"]
      = DART_BM_LCP_COMPARE_DART_SIMD_FORCE_SCALAR ? 1.0 : 0.0;
  state.counters["build_cuda_enabled"]
      = DART_BM_LCP_COMPARE_DART_ENABLE_EXPERIMENTAL_CUDA ? 1.0 : 0.0;
  state.counters["has_simulation"]
      = DART_BM_LCP_COMPARE_HAS_SIMULATION ? 1.0 : 0.0;
}

void AddSolverIdentityCounters(
    benchmark::State& state, const dart::test::LcpSolverManifestEntry& solver)
{
  state.counters["solver_identity_schema_version"]
      = static_cast<double>(dart::test::kLcpSolverIdentitySchemaVersion);
  state.counters["solver_manifest_index"]
      = static_cast<double>(dart::test::getLcpSolverManifestIndex(solver.name));
  state.counters["solver_family_pivoting"]
      = solver.family == "Pivoting" ? 1.0 : 0.0;
  state.counters["solver_family_projection"]
      = solver.family == "Projection" ? 1.0 : 0.0;
  state.counters["solver_family_newton"]
      = solver.family == "Newton" ? 1.0 : 0.0;
  state.counters["solver_family_other"] = solver.family == "Other" ? 1.0 : 0.0;
}

void AddSolverProblemSupportCounters(
    benchmark::State& state,
    const dart::math::LcpSolver& solver,
    const LcpProblem& problem)
{
  const auto problemType
      = problem.getType(solver.getDefaultOptions().absoluteTolerance);

  state.counters["solver_supports_standard"]
      = solver.supportsStandardLcp() ? 1.0 : 0.0;
  state.counters["solver_supports_boxed"]
      = solver.supportsBoxedLcp() ? 1.0 : 0.0;
  state.counters["solver_supports_friction_index"]
      = solver.supportsFrictionIndex() ? 1.0 : 0.0;
  state.counters["solver_supports_problem"]
      = solver.supportsProblem(problem) ? 1.0 : 0.0;

  state.counters["problem_type_standard"]
      = problemType == dart::math::LcpProblemType::Standard ? 1.0 : 0.0;
  state.counters["problem_type_boxed"]
      = problemType == dart::math::LcpProblemType::Boxed ? 1.0 : 0.0;
  state.counters["problem_type_friction_index"]
      = problemType == dart::math::LcpProblemType::FrictionIndex ? 1.0 : 0.0;
  state.counters["problem_type_invalid"]
      = problemType == dart::math::LcpProblemType::Invalid ? 1.0 : 0.0;
}

int ChooseWorkerCount(Eigen::Index problemSize)
{
  const auto hardwareWorkers = std::thread::hardware_concurrency();
  const int availableWorkers
      = hardwareWorkers > 0 ? static_cast<int>(hardwareWorkers) : 2;
  return std::max(
      1, std::min({availableWorkers, 8, static_cast<int>(problemSize)}));
}

ValidationCounters ComputeValidationCountersSerial(
    const LcpProblem& problem,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& loEff,
    const Eigen::VectorXd& hiEff,
    double tol)
{
  return ComputeValidationRange(problem, x, loEff, hiEff, tol, 0, x.size());
}

ValidationCounters ComputeValidationCountersThreaded(
    const LcpProblem& problem,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& loEff,
    const Eigen::VectorXd& hiEff,
    double tol,
    int workerCount)
{
  std::vector<ValidationCounters> partials(workerCount);
  std::vector<std::thread> workers;
  workers.reserve(workerCount);

  const Eigen::Index n = x.size();
  const Eigen::Index chunkSize
      = (n + static_cast<Eigen::Index>(workerCount) - 1) / workerCount;

  for (int worker = 0; worker < workerCount; ++worker) {
    const Eigen::Index begin = static_cast<Eigen::Index>(worker) * chunkSize;
    const Eigen::Index end = std::min(n, begin + chunkSize);
    workers.emplace_back([&, worker, begin, end]() {
      partials[worker]
          = ComputeValidationRange(problem, x, loEff, hiEff, tol, begin, end);
    });
  }

  for (auto& worker : workers) {
    worker.join();
  }

  ValidationCounters counters;
  for (const auto& partial : partials) {
    MergeValidationCounters(counters, partial);
  }
  return counters;
}

Eigen::VectorXd SolveBenchmarkProblem(const LcpProblem& problem)
{
  dart::math::DantzigSolver solver;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  auto options = MakeBenchmarkOptions(100);
  options.validateSolution = false;
  solver.solve(problem, x, options);
  return x;
}

void AddValidationBenchmarkCounters(
    benchmark::State& state,
    const LcpProblem& problem,
    const Eigen::VectorXd& x,
    const LcpOptions& options,
    const ValidationCounters& counters,
    int workerCount,
    const std::string& label)
{
  const auto check = dart::test::CheckLcpSolution(problem, x, options);
  state.counters["worker_count"] = workerCount;
  state.counters["residual"] = counters.residual;
  state.counters["complementarity"] = counters.complementarity;
  state.counters["bound_violation"] = counters.boundViolation;
  state.counters["contract_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetLabel(label);
}

void RunBenchmarkWithSolver(
    benchmark::State& state,
    dart::math::LcpSolver& solver,
    const LcpProblem& problem,
    const LcpOptions& options,
    const std::string& label)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
  dart::math::LcpResult lastResult;

  for (auto _ : state) {
    x.setZero();
    lastResult = solver.solve(problem, x, options);
    benchmark::DoNotOptimize(lastResult.status);
    benchmark::DoNotOptimize(x.data());
  }

  const auto check = dart::test::CheckLcpSolution(problem, x, options);
  state.counters["iterations"] = lastResult.iterations;
  state.counters["residual"] = check.residual;
  state.counters["complementarity"] = check.complementarity;
  state.counters["bound_violation"] = check.boundViolation;
  state.counters["contract_ok"] = check.ok ? 1.0 : 0.0;
  state.counters["problem_size"] = problem.b.size();
  const auto frictionIndexContactCount = problem.getFrictionIndexContactCount();
  if (frictionIndexContactCount > 0) {
    state.counters["contact_count"]
        = static_cast<double>(frictionIndexContactCount);
  }
  AddSolverProblemSupportCounters(state, solver, problem);
  AddBackendBuildCounters(state);
  state.SetLabel(label);
}

template <typename Solver>
void RunBenchmark(
    benchmark::State& state,
    const LcpProblem& problem,
    const LcpOptions& options,
    const std::string& label)
{
  Solver solver;
  RunBenchmarkWithSolver(state, solver, problem, options, label);
}

static void BM_LcpValidation_Serial_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 5000u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  const Eigen::VectorXd x = SolveBenchmarkProblem(problem);

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  benchmark::DoNotOptimize(
      dart::math::detail::computeEffectiveBounds(
          problem.lo, problem.hi, problem.findex, x, loEff, hiEff, &message));

  ValidationCounters counters;
  for (auto _ : state) {
    counters = ComputeValidationCountersSerial(
        problem, x, loEff, hiEff, options.complementarityTolerance);
    benchmark::DoNotOptimize(counters.residual);
  }
  counters = ComputeValidationCountersSerial(
      problem, x, loEff, hiEff, options.complementarityTolerance);

  AddValidationBenchmarkCounters(
      state,
      problem,
      x,
      options,
      counters,
      1,
      MakeLabel("SerialValidation", "FrictionIndex"));
}

static void BM_LcpValidation_Threaded_FrictionIndex(benchmark::State& state)
{
  const int numContacts = static_cast<int>(state.range(0));
  const auto problem = MakeFrictionIndexProblem(
      numContacts, 5000u + static_cast<unsigned>(numContacts));
  const auto options = MakeBenchmarkOptions(100);
  const Eigen::VectorXd x = SolveBenchmarkProblem(problem);
  const int workerCount = ChooseWorkerCount(x.size());

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  benchmark::DoNotOptimize(
      dart::math::detail::computeEffectiveBounds(
          problem.lo, problem.hi, problem.findex, x, loEff, hiEff, &message));

  ValidationCounters counters;
  for (auto _ : state) {
    counters = ComputeValidationCountersThreaded(
        problem,
        x,
        loEff,
        hiEff,
        options.complementarityTolerance,
        workerCount);
    benchmark::DoNotOptimize(counters.residual);
  }
  counters = ComputeValidationCountersThreaded(
      problem, x, loEff, hiEff, options.complementarityTolerance, workerCount);

  AddValidationBenchmarkCounters(
      state,
      problem,
      x,
      options,
      counters,
      workerCount,
      MakeLabel("ThreadedValidation", "FrictionIndex"));
}

enum class JacobiThreadingProblemKind
{
  DenseSpd,
  BandedSpd
};

double CountBandedSpdNonzeros(const int problemSize)
{
  double nonzeros = static_cast<double>(problemSize);
  if (problemSize > 1) {
    nonzeros += 2.0 * static_cast<double>(problemSize - 1);
  }
  if (problemSize > 2) {
    nonzeros += 2.0 * static_cast<double>(problemSize - 2);
  }
  return nonzeros;
}

const char* getJacobiThreadingProblemLabel(
    const JacobiThreadingProblemKind kind)
{
  switch (kind) {
    case JacobiThreadingProblemKind::DenseSpd:
      return "StandardDenseSpd";
    case JacobiThreadingProblemKind::BandedSpd:
      return "StandardBandedSpd";
  }
  return "StandardUnknown";
}

LcpProblem MakeJacobiThreadingProblem(
    const JacobiThreadingProblemKind kind,
    const int problemSize,
    const unsigned seed)
{
  switch (kind) {
    case JacobiThreadingProblemKind::DenseSpd:
      return MakeStandardSpdProblem(problemSize, seed);
    case JacobiThreadingProblemKind::BandedSpd:
      return MakeStandardBandedSpdProblem(problemSize, seed);
  }
  return MakeStandardSpdProblem(problemSize, seed);
}

void RunJacobiSolverThreadingBenchmark(
    benchmark::State& state, const JacobiThreadingProblemKind kind)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int requestedWorkerCount = static_cast<int>(state.range(1));
  const int workerCount = std::max(
      1, std::min(requestedWorkerCount, static_cast<int>(problemSize)));

  const auto problem = MakeJacobiThreadingProblem(
      kind, problemSize, 60'001u + static_cast<unsigned>(problemSize));
  auto options = MakeBenchmarkOptions(500);
  dart::math::JacobiSolver::Parameters params;
  params.workerThreads = workerCount;
  options.customOptions = &params;

  dart::math::JacobiSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          workerCount == 1 ? "JacobiSerial" : "JacobiThreaded",
          getJacobiThreadingProblemLabel(kind)));

  const double denseEntries
      = static_cast<double>(problemSize) * static_cast<double>(problemSize);
  const double nonzeroEntries = kind == JacobiThreadingProblemKind::BandedSpd
                                    ? CountBandedSpdNonzeros(problemSize)
                                    : denseEntries;

  state.counters["worker_count"] = workerCount;
  state.counters["solver_internal_threads"] = workerCount;
  state.counters["jacobi_threading_problem_kind"]
      = static_cast<double>(static_cast<int>(kind));
  state.counters["jacobi_threading_banded_spd"]
      = kind == JacobiThreadingProblemKind::BandedSpd ? 1.0 : 0.0;
  state.counters["band_half_width"]
      = kind == JacobiThreadingProblemKind::BandedSpd
            ? 2.0
            : static_cast<double>(std::max(0, problemSize - 1));
  state.counters["matrix_nonzero_entries"] = nonzeroEntries;
  state.counters["matrix_density"] = nonzeroEntries / denseEntries;
}

static void BM_LcpJacobiSolverThreading_Standard(benchmark::State& state)
{
  RunJacobiSolverThreadingBenchmark(
      state, JacobiThreadingProblemKind::DenseSpd);
}

static void BM_LcpJacobiSolverThreadingBanded_Standard(benchmark::State& state)
{
  RunJacobiSolverThreadingBenchmark(
      state, JacobiThreadingProblemKind::BandedSpd);
}

void RunRedBlackGaussSeidelThreadingBenchmark(benchmark::State& state)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int requestedWorkerCount = static_cast<int>(state.range(1));
  const int largestColorSize = (problemSize + 1) / 2;
  const int workerCount = std::max(
      1, std::min(requestedWorkerCount, std::max(1, largestColorSize)));

  const auto problem = MakeStandardBandedSpdProblem(
      problemSize, 70'001u + static_cast<unsigned>(problemSize));
  auto options = MakeBenchmarkOptions(300);
  dart::math::RedBlackGaussSeidelSolver::Parameters params;
  params.workerThreads = workerCount;
  options.customOptions = &params;

  dart::math::RedBlackGaussSeidelSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          workerCount == 1 ? "RedBlackGaussSeidelSerial"
                           : "RedBlackGaussSeidelThreaded",
          "StandardBandedSpd"));

  const double problemSizeValue = static_cast<double>(problemSize);
  const double nonzeroEntries = CountBandedSpdNonzeros(problemSize);
  state.counters["worker_count"] = workerCount;
  state.counters["solver_internal_threads"] = workerCount;
  state.counters["red_black_threading"] = 1.0;
  state.counters["red_black_color_count"] = 2.0;
  state.counters["red_black_red_rows"] = std::ceil(problemSizeValue / 2.0);
  state.counters["red_black_black_rows"] = std::floor(problemSizeValue / 2.0);
  state.counters["red_black_threaded_color_updates"]
      = workerCount > 1 ? 1.0 : 0.0;
  state.counters["band_half_width"] = 2.0;
  state.counters["matrix_nonzero_entries"] = nonzeroEntries;
  state.counters["matrix_density"]
      = nonzeroEntries / (problemSizeValue * problemSizeValue);
}

static void BM_LcpRedBlackGaussSeidelSolverThreadingBanded_Standard(
    benchmark::State& state)
{
  RunRedBlackGaussSeidelThreadingBenchmark(state);
}

void RunBlockedJacobiThreadingBenchmark(benchmark::State& state)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int requestedWorkerCount = static_cast<int>(state.range(1));
  const int workerCount
      = std::max(1, std::min(requestedWorkerCount, problemSize));

  const auto problem = MakeStandardBandedSpdProblem(
      problemSize, 72'001u + static_cast<unsigned>(problemSize));
  auto options = MakeBenchmarkOptions(300);
  dart::math::BlockedJacobiSolver::Parameters params;
  params.workerThreads = workerCount;
  options.customOptions = &params;

  dart::math::BlockedJacobiSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          workerCount == 1 ? "BlockedJacobiSerial" : "BlockedJacobiThreaded",
          "StandardBandedSpd"));

  const double problemSizeValue = static_cast<double>(problemSize);
  const double nonzeroEntries = CountBandedSpdNonzeros(problemSize);
  state.counters["worker_count"] = workerCount;
  state.counters["solver_internal_threads"] = workerCount;
  state.counters["blocked_jacobi_threading"] = 1.0;
  state.counters["blocked_jacobi_auto_singleton_blocks"] = 1.0;
  state.counters["block_count"] = problemSize;
  state.counters["min_block_size"] = 1.0;
  state.counters["max_block_size"] = 1.0;
  state.counters["blocked_jacobi_threaded_block_updates"]
      = workerCount > 1 ? 1.0 : 0.0;
  state.counters["band_half_width"] = 2.0;
  state.counters["matrix_nonzero_entries"] = nonzeroEntries;
  state.counters["matrix_density"]
      = nonzeroEntries / (problemSizeValue * problemSizeValue);
}

static void BM_LcpBlockedJacobiSolverThreadingBanded_Standard(
    benchmark::State& state)
{
  RunBlockedJacobiThreadingBenchmark(state);
}

enum class BenchmarkProblemFamily
{
  Standard,
  Boxed,
  FrictionIndex
};

enum class RelaxationSweepKind
{
  Under,
  Plain,
  Over
};

enum class NewtonWarmStartMode
{
  None,
  Pgs,
  GradientDescent,
  PgsThenGradient
};

enum class ShockPropagationLayerProfile
{
  SingleLayer,
  TwoLayers,
  SerialLayers
};

enum class BlockPartitionProfile
{
  FullBlock,
  UniformThree,
  AutoFindex,
  ContactBlocks
};

enum class StandardSpdProblemKind
{
  DenseSpd,
  BandedSpd,
  MildIllConditioned,
  NearSingular
};

constexpr int kNewtonWarmStartPgsIterations = 5;
constexpr int kNewtonWarmStartGradientIterations = 5;
constexpr std::array<int, 3> kNewtonWarmStartProblemSizes{{32, 64, 128}};
constexpr int kNewtonWarmStartBatchSize = 4;

struct RelaxationSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  RelaxationSweepKind relaxationKind;
  double relaxation;
  std::string_view relaxationLabel;
};

struct BoxedSsnLineSearchSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  int maxLineSearchSteps;
  double stepReduction;
  std::string_view lineSearchLabel;
};

struct PivotingScaleSweepCase
{
  std::string_view solverName;
  BenchmarkProblemFamily family;
  int problemArg;
  std::string_view problemLabel;
};

struct BlockPartitionSweepCase
{
  std::string_view solverName;
  BenchmarkProblemFamily family;
  int problemArg;
  BlockPartitionProfile profile;
  std::string_view profileLabel;
};

struct ApgdRestartSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  bool adaptiveRestart;
  int restartCheckInterval;
  std::string_view restartPolicyLabel;
};

struct TgsIterationBudgetSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  int maxIterations;
  std::string_view iterationBudgetLabel;
};

struct NncgPgsIterationsSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  int pgsIterations;
  std::string_view pgsIterationsLabel;
};

struct SubspacePgsIterationsSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  int pgsIterations;
  std::string_view pgsIterationsLabel;
};

struct ShockPropagationLayerSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  ShockPropagationLayerProfile profile;
  std::string_view profileLabel;
};

struct MprgpSpdCheckSweepCase
{
  StandardSpdProblemKind kind;
  int problemSize;
  bool checkPositiveDefinite;
  std::string_view kindLabel;
  std::string_view pdCheckLabel;
  unsigned seed;
};

struct InteriorPointPathSweepCase
{
  StandardSpdProblemKind kind;
  int problemSize;
  double sigma;
  double stepScale;
  std::string_view kindLabel;
  std::string_view sigmaLabel;
  std::string_view stepScaleLabel;
  unsigned seed;
};

struct AdmmRhoSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  double rhoInit;
  bool adaptiveRho;
  std::string_view rhoLabel;
  std::string_view policyLabel;
};

struct SapRegularizationSweepCase
{
  BenchmarkProblemFamily family;
  int problemArg;
  double regularization;
  std::string_view regularizationLabel;
};

constexpr std::array<std::string_view, 3> kContactComparisonSolverNames{
    "Admm",
    "Sap",
    "BoxedSemiSmoothNewton",
};

constexpr std::array<std::string_view, 9> kContactNormalStandardSolverNames{
    "Dantzig",
    "Lemke",
    "Baraff",
    "Direct",
    "MinimumMapNewton",
    "FischerBurmeisterNewton",
    "PenalizedFischerBurmeisterNewton",
    "InteriorPoint",
    "MPRGP",
};

constexpr std::array<StaggeringContactPipelineSweepCase, 24>
    kContactNormalStandardSweepCases{{
        {StaggeringContactPipelineKind::WorldSeparated, 1, "WorldSeparated1"},
        {StaggeringContactPipelineKind::WorldSeparated, 2, "WorldSeparated2"},
        {StaggeringContactPipelineKind::WorldSeparated, 4, "WorldSeparated4"},
        {StaggeringContactPipelineKind::WorldSeparated, 8, "WorldSeparated8"},
        {StaggeringContactPipelineKind::WorldSeparated, 16, "WorldSeparated16"},
        {StaggeringContactPipelineKind::WorldSeparated, 32, "WorldSeparated32"},
        {StaggeringContactPipelineKind::WorldStack, 2, "WorldStack2"},
        {StaggeringContactPipelineKind::WorldStack, 3, "WorldStack3"},
        {StaggeringContactPipelineKind::WorldStack, 5, "WorldStack5"},
        {StaggeringContactPipelineKind::WorldStack, 8, "WorldStack8"},
        {StaggeringContactPipelineKind::WorldStack, 16, "WorldStack16"},
        {StaggeringContactPipelineKind::WorldStack, 32, "WorldStack32"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         4,
         "ArticulatedGround4"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         8,
         "ArticulatedGround8"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         16,
         "ArticulatedGround16"},
        {StaggeringContactPipelineKind::ArticulatedGround,
         32,
         "ArticulatedGround32"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         4,
         "ArticulatedRigidImpact4"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         8,
         "ArticulatedRigidImpact8"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         16,
         "ArticulatedRigidImpact16"},
        {StaggeringContactPipelineKind::ArticulatedRigidImpact,
         32,
         "ArticulatedRigidImpact32"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         4,
         "ArticulatedCrossLinkImpact4"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         8,
         "ArticulatedCrossLinkImpact8"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         16,
         "ArticulatedCrossLinkImpact16"},
        {StaggeringContactPipelineKind::ArticulatedCrossLinkImpact,
         32,
         "ArticulatedCrossLinkImpact32"},
    }};

constexpr std::array<RelaxationSweepCase, 12> kRelaxationSweepCases{{
    {BenchmarkProblemFamily::Standard,
     48,
     RelaxationSweepKind::Under,
     0.5,
     "Relaxation0_5"},
    {BenchmarkProblemFamily::Standard,
     48,
     RelaxationSweepKind::Plain,
     1.0,
     "Relaxation1_0"},
    {BenchmarkProblemFamily::Standard,
     48,
     RelaxationSweepKind::Over,
     1.3,
     "Relaxation1_3"},
    {BenchmarkProblemFamily::Boxed,
     24,
     RelaxationSweepKind::Under,
     0.5,
     "Relaxation0_5"},
    {BenchmarkProblemFamily::Boxed,
     24,
     RelaxationSweepKind::Plain,
     1.0,
     "Relaxation1_0"},
    {BenchmarkProblemFamily::Boxed,
     24,
     RelaxationSweepKind::Over,
     1.3,
     "Relaxation1_3"},
    {BenchmarkProblemFamily::FrictionIndex,
     8,
     RelaxationSweepKind::Under,
     0.5,
     "Relaxation0_5"},
    {BenchmarkProblemFamily::FrictionIndex,
     8,
     RelaxationSweepKind::Plain,
     1.0,
     "Relaxation1_0"},
    {BenchmarkProblemFamily::FrictionIndex,
     8,
     RelaxationSweepKind::Over,
     1.3,
     "Relaxation1_3"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     RelaxationSweepKind::Under,
     0.5,
     "Contacts16Relaxation0_5"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     RelaxationSweepKind::Plain,
     1.0,
     "Contacts16Relaxation1_0"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     RelaxationSweepKind::Over,
     1.3,
     "Contacts16Relaxation1_3"},
}};

constexpr std::array<BoxedSsnLineSearchSweepCase, 12>
    kBoxedSsnLineSearchSweepCases{{
        {BenchmarkProblemFamily::Standard, 48, 10, 0.5, "DefaultSearch"},
        {BenchmarkProblemFamily::Standard, 48, 20, 0.5, "MoreSteps"},
        {BenchmarkProblemFamily::Standard, 48, 20, 0.8, "GentleReduction"},
        {BenchmarkProblemFamily::Boxed, 24, 10, 0.5, "DefaultSearch"},
        {BenchmarkProblemFamily::Boxed, 24, 20, 0.5, "MoreSteps"},
        {BenchmarkProblemFamily::Boxed, 24, 20, 0.8, "GentleReduction"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 10, 0.5, "DefaultSearch"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 20, 0.5, "MoreSteps"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 20, 0.8, "GentleReduction"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         10,
         0.5,
         "Contacts16DefaultSearch"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         20,
         0.5,
         "Contacts16MoreSteps"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         20,
         0.8,
         "Contacts16GentleReduction"},
    }};

constexpr std::array<PivotingScaleSweepCase, 15> kPivotingScaleSweepCases{{
    {"Direct", BenchmarkProblemFamily::Standard, 2, "Rows2"},
    {"Direct", BenchmarkProblemFamily::Standard, 3, "Rows3"},
    {"Lemke", BenchmarkProblemFamily::Standard, 8, "Rows8"},
    {"Lemke", BenchmarkProblemFamily::Standard, 16, "Rows16"},
    {"Baraff", BenchmarkProblemFamily::Standard, 8, "Rows8"},
    {"Baraff", BenchmarkProblemFamily::Standard, 16, "Rows16"},
    {"Dantzig", BenchmarkProblemFamily::Standard, 8, "Rows8"},
    {"Dantzig", BenchmarkProblemFamily::Standard, 16, "Rows16"},
    {"Dantzig", BenchmarkProblemFamily::Standard, 32, "Rows32"},
    {"Dantzig", BenchmarkProblemFamily::Boxed, 12, "Rows12"},
    {"Dantzig", BenchmarkProblemFamily::Boxed, 24, "Rows24"},
    {"Dantzig", BenchmarkProblemFamily::Boxed, 48, "Rows48"},
    {"Dantzig", BenchmarkProblemFamily::FrictionIndex, 4, "Contacts4"},
    {"Dantzig", BenchmarkProblemFamily::FrictionIndex, 8, "Contacts8"},
    {"Dantzig", BenchmarkProblemFamily::FrictionIndex, 16, "Contacts16"},
}};

constexpr std::array<BlockPartitionSweepCase, 16> kBlockPartitionSweepCases{{
    {"BGS",
     BenchmarkProblemFamily::Standard,
     12,
     BlockPartitionProfile::FullBlock,
     "FullBlock"},
    {"BGS",
     BenchmarkProblemFamily::Standard,
     12,
     BlockPartitionProfile::UniformThree,
     "Blocks3"},
    {"BGS",
     BenchmarkProblemFamily::Boxed,
     12,
     BlockPartitionProfile::FullBlock,
     "FullBlock"},
    {"BGS",
     BenchmarkProblemFamily::Boxed,
     12,
     BlockPartitionProfile::UniformThree,
     "Blocks3"},
    {"BGS",
     BenchmarkProblemFamily::FrictionIndex,
     4,
     BlockPartitionProfile::AutoFindex,
     "AutoFindex"},
    {"BGS",
     BenchmarkProblemFamily::FrictionIndex,
     4,
     BlockPartitionProfile::ContactBlocks,
     "ContactBlocks"},
    {"BGS",
     BenchmarkProblemFamily::FrictionIndex,
     8,
     BlockPartitionProfile::AutoFindex,
     "Contacts8AutoFindex"},
    {"BGS",
     BenchmarkProblemFamily::FrictionIndex,
     8,
     BlockPartitionProfile::ContactBlocks,
     "Contacts8ContactBlocks"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::Standard,
     12,
     BlockPartitionProfile::FullBlock,
     "FullBlock"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::Standard,
     12,
     BlockPartitionProfile::UniformThree,
     "Blocks3"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::Boxed,
     12,
     BlockPartitionProfile::FullBlock,
     "FullBlock"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::Boxed,
     12,
     BlockPartitionProfile::UniformThree,
     "Blocks3"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::FrictionIndex,
     4,
     BlockPartitionProfile::AutoFindex,
     "AutoFindex"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::FrictionIndex,
     4,
     BlockPartitionProfile::ContactBlocks,
     "ContactBlocks"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::FrictionIndex,
     8,
     BlockPartitionProfile::AutoFindex,
     "Contacts8AutoFindex"},
    {"BlockedJacobi",
     BenchmarkProblemFamily::FrictionIndex,
     8,
     BlockPartitionProfile::ContactBlocks,
     "Contacts8ContactBlocks"},
}};

constexpr std::array<ApgdRestartSweepCase, 12> kApgdRestartSweepCases{{
    {BenchmarkProblemFamily::Standard, 48, true, 0, "AdaptiveEveryIter"},
    {BenchmarkProblemFamily::Standard, 48, true, 5, "AdaptiveEvery5"},
    {BenchmarkProblemFamily::Standard, 48, false, 0, "NoRestart"},
    {BenchmarkProblemFamily::Boxed, 24, true, 0, "AdaptiveEveryIter"},
    {BenchmarkProblemFamily::Boxed, 24, true, 5, "AdaptiveEvery5"},
    {BenchmarkProblemFamily::Boxed, 24, false, 0, "NoRestart"},
    {BenchmarkProblemFamily::FrictionIndex, 8, true, 0, "AdaptiveEveryIter"},
    {BenchmarkProblemFamily::FrictionIndex, 8, true, 5, "AdaptiveEvery5"},
    {BenchmarkProblemFamily::FrictionIndex, 8, false, 0, "NoRestart"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     true,
     0,
     "Contacts16AdaptiveEveryIter"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     true,
     5,
     "Contacts16AdaptiveEvery5"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     false,
     0,
     "Contacts16NoRestart"},
}};

constexpr std::array<TgsIterationBudgetSweepCase, 12>
    kTgsIterationBudgetSweepCases{{
        {BenchmarkProblemFamily::Standard, 48, 10, "Iter10"},
        {BenchmarkProblemFamily::Standard, 48, 50, "Iter50"},
        {BenchmarkProblemFamily::Standard, 48, 100, "Iter100"},
        {BenchmarkProblemFamily::Boxed, 24, 10, "Iter10"},
        {BenchmarkProblemFamily::Boxed, 24, 50, "Iter50"},
        {BenchmarkProblemFamily::Boxed, 24, 100, "Iter100"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 10, "Iter10"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 50, "Iter50"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 100, "Iter100"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 10, "Contacts16Iter10"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 50, "Contacts16Iter50"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 100, "Contacts16Iter100"},
    }};

constexpr std::array<NncgPgsIterationsSweepCase, 12>
    kNncgPgsIterationsSweepCases{{
        {BenchmarkProblemFamily::Standard, 48, 1, "PgsIter1"},
        {BenchmarkProblemFamily::Standard, 48, 2, "PgsIter2"},
        {BenchmarkProblemFamily::Standard, 48, 5, "PgsIter5"},
        {BenchmarkProblemFamily::Boxed, 24, 1, "PgsIter1"},
        {BenchmarkProblemFamily::Boxed, 24, 2, "PgsIter2"},
        {BenchmarkProblemFamily::Boxed, 24, 5, "PgsIter5"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 1, "PgsIter1"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 2, "PgsIter2"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 5, "PgsIter5"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 1, "Contacts16PgsIter1"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 2, "Contacts16PgsIter2"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 5, "Contacts16PgsIter5"},
    }};

constexpr std::array<SubspacePgsIterationsSweepCase, 12>
    kSubspacePgsIterationsSweepCases{{
        {BenchmarkProblemFamily::Standard, 48, 1, "PgsIter1"},
        {BenchmarkProblemFamily::Standard, 48, 3, "PgsIter3"},
        {BenchmarkProblemFamily::Standard, 48, 5, "PgsIter5"},
        {BenchmarkProblemFamily::Boxed, 24, 1, "PgsIter1"},
        {BenchmarkProblemFamily::Boxed, 24, 3, "PgsIter3"},
        {BenchmarkProblemFamily::Boxed, 24, 5, "PgsIter5"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 1, "PgsIter1"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 3, "PgsIter3"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 5, "PgsIter5"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 1, "Contacts16PgsIter1"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 3, "Contacts16PgsIter3"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 5, "Contacts16PgsIter5"},
    }};

constexpr std::array<ShockPropagationLayerSweepCase, 12>
    kShockPropagationLayerSweepCases{{
        {BenchmarkProblemFamily::Standard,
         48,
         ShockPropagationLayerProfile::SingleLayer,
         "SingleLayer"},
        {BenchmarkProblemFamily::Standard,
         48,
         ShockPropagationLayerProfile::TwoLayers,
         "TwoLayers"},
        {BenchmarkProblemFamily::Standard,
         48,
         ShockPropagationLayerProfile::SerialLayers,
         "SerialLayers"},
        {BenchmarkProblemFamily::Boxed,
         24,
         ShockPropagationLayerProfile::SingleLayer,
         "SingleLayer"},
        {BenchmarkProblemFamily::Boxed,
         24,
         ShockPropagationLayerProfile::TwoLayers,
         "TwoLayers"},
        {BenchmarkProblemFamily::Boxed,
         24,
         ShockPropagationLayerProfile::SerialLayers,
         "SerialLayers"},
        {BenchmarkProblemFamily::FrictionIndex,
         8,
         ShockPropagationLayerProfile::SingleLayer,
         "SingleLayer"},
        {BenchmarkProblemFamily::FrictionIndex,
         8,
         ShockPropagationLayerProfile::TwoLayers,
         "TwoLayers"},
        {BenchmarkProblemFamily::FrictionIndex,
         8,
         ShockPropagationLayerProfile::SerialLayers,
         "SerialLayers"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         ShockPropagationLayerProfile::SingleLayer,
         "Contacts16SingleLayer"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         ShockPropagationLayerProfile::TwoLayers,
         "Contacts16TwoLayers"},
        {BenchmarkProblemFamily::FrictionIndex,
         16,
         ShockPropagationLayerProfile::SerialLayers,
         "Contacts16SerialLayers"},
    }};

constexpr std::array<MprgpSpdCheckSweepCase, 12> kMprgpSpdCheckSweepCases{{
    {StandardSpdProblemKind::DenseSpd,
     32,
     true,
     "DenseSpd",
     "PdCheckOn",
     0x6010u},
    {StandardSpdProblemKind::DenseSpd,
     32,
     false,
     "DenseSpd",
     "PdCheckOff",
     0x6010u},
    {StandardSpdProblemKind::DenseSpd,
     64,
     true,
     "DenseSpd",
     "PdCheckOn",
     0x6011u},
    {StandardSpdProblemKind::BandedSpd,
     64,
     true,
     "BandedSpd",
     "PdCheckOn",
     0x6012u},
    {StandardSpdProblemKind::BandedSpd,
     64,
     false,
     "BandedSpd",
     "PdCheckOff",
     0x6012u},
    {StandardSpdProblemKind::MildIllConditioned,
     32,
     true,
     "MildIllConditioned",
     "PdCheckOn",
     0x6013u},
    {StandardSpdProblemKind::MildIllConditioned,
     32,
     false,
     "MildIllConditioned",
     "PdCheckOff",
     0x6013u},
    {StandardSpdProblemKind::NearSingular,
     8,
     true,
     "NearSingular",
     "PdCheckOn",
     0x6014u},
    {StandardSpdProblemKind::NearSingular,
     8,
     false,
     "NearSingular",
     "PdCheckOff",
     0x6014u},
    {StandardSpdProblemKind::DenseSpd,
     128,
     true,
     "DenseSpd",
     "PdCheckOn",
     0x6015u},
    {StandardSpdProblemKind::BandedSpd,
     128,
     true,
     "BandedSpd",
     "PdCheckOn",
     0x6016u},
    {StandardSpdProblemKind::NearSingular,
     16,
     true,
     "NearSingular",
     "PdCheckOn",
     0x6017u},
}};

constexpr std::array<InteriorPointPathSweepCase, 12>
    kInteriorPointPathSweepCases{{
        {StandardSpdProblemKind::DenseSpd,
         32,
         0.1,
         0.99,
         "DenseSpd",
         "Sigma0_1",
         "Step0_99",
         0x7010u},
        {StandardSpdProblemKind::DenseSpd,
         32,
         0.3,
         0.99,
         "DenseSpd",
         "Sigma0_3",
         "Step0_99",
         0x7010u},
        {StandardSpdProblemKind::DenseSpd,
         32,
         0.1,
         0.75,
         "DenseSpd",
         "Sigma0_1",
         "Step0_75",
         0x7010u},
        {StandardSpdProblemKind::DenseSpd,
         64,
         0.1,
         0.99,
         "DenseSpd",
         "Sigma0_1",
         "Step0_99",
         0x7011u},
        {StandardSpdProblemKind::BandedSpd,
         64,
         0.1,
         0.99,
         "BandedSpd",
         "Sigma0_1",
         "Step0_99",
         0x7012u},
        {StandardSpdProblemKind::BandedSpd,
         64,
         0.3,
         0.99,
         "BandedSpd",
         "Sigma0_3",
         "Step0_99",
         0x7012u},
        {StandardSpdProblemKind::MildIllConditioned,
         32,
         0.1,
         0.99,
         "MildIllConditioned",
         "Sigma0_1",
         "Step0_99",
         0x7013u},
        {StandardSpdProblemKind::MildIllConditioned,
         32,
         0.3,
         0.99,
         "MildIllConditioned",
         "Sigma0_3",
         "Step0_99",
         0x7013u},
        {StandardSpdProblemKind::NearSingular,
         8,
         0.1,
         0.99,
         "NearSingular",
         "Sigma0_1",
         "Step0_99",
         0x7014u},
        {StandardSpdProblemKind::DenseSpd,
         128,
         0.1,
         0.99,
         "DenseSpd",
         "Sigma0_1",
         "Step0_99",
         0x7015u},
        {StandardSpdProblemKind::BandedSpd,
         128,
         0.1,
         0.99,
         "BandedSpd",
         "Sigma0_1",
         "Step0_99",
         0x7016u},
        {StandardSpdProblemKind::NearSingular,
         16,
         0.1,
         0.99,
         "NearSingular",
         "Sigma0_1",
         "Step0_99",
         0x7017u},
    }};

constexpr std::array<AdmmRhoSweepCase, 24> kAdmmRhoSweepCases{{
    {BenchmarkProblemFamily::Standard, 48, 0.5, false, "Rho0_5", "Fixed"},
    {BenchmarkProblemFamily::Standard, 48, 1.0, false, "Rho1_0", "Fixed"},
    {BenchmarkProblemFamily::Standard, 48, 4.0, false, "Rho4_0", "Fixed"},
    {BenchmarkProblemFamily::Standard, 48, 0.5, true, "Rho0_5", "Adaptive"},
    {BenchmarkProblemFamily::Standard, 48, 1.0, true, "Rho1_0", "Adaptive"},
    {BenchmarkProblemFamily::Standard, 48, 4.0, true, "Rho4_0", "Adaptive"},
    {BenchmarkProblemFamily::Boxed, 24, 0.5, false, "Rho0_5", "Fixed"},
    {BenchmarkProblemFamily::Boxed, 24, 1.0, false, "Rho1_0", "Fixed"},
    {BenchmarkProblemFamily::Boxed, 24, 4.0, false, "Rho4_0", "Fixed"},
    {BenchmarkProblemFamily::Boxed, 24, 0.5, true, "Rho0_5", "Adaptive"},
    {BenchmarkProblemFamily::Boxed, 24, 1.0, true, "Rho1_0", "Adaptive"},
    {BenchmarkProblemFamily::Boxed, 24, 4.0, true, "Rho4_0", "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 0.5, false, "Rho0_5", "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 1.0, false, "Rho1_0", "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 4.0, false, "Rho4_0", "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 0.5, true, "Rho0_5", "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 1.0, true, "Rho1_0", "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex, 8, 4.0, true, "Rho4_0", "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     0.5,
     false,
     "Contacts16Rho0_5",
     "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     1.0,
     false,
     "Contacts16Rho1_0",
     "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     4.0,
     false,
     "Contacts16Rho4_0",
     "Fixed"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     0.5,
     true,
     "Contacts16Rho0_5",
     "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     1.0,
     true,
     "Contacts16Rho1_0",
     "Adaptive"},
    {BenchmarkProblemFamily::FrictionIndex,
     16,
     4.0,
     true,
     "Contacts16Rho4_0",
     "Adaptive"},
}};

constexpr std::array<SapRegularizationSweepCase, 12>
    kSapRegularizationSweepCases{{
        {BenchmarkProblemFamily::Standard, 48, 1e-6, "Reg1e_6"},
        {BenchmarkProblemFamily::Standard, 48, 1e-5, "Reg1e_5"},
        {BenchmarkProblemFamily::Standard, 48, 1e-4, "Reg1e_4"},
        {BenchmarkProblemFamily::Boxed, 24, 1e-6, "Reg1e_6"},
        {BenchmarkProblemFamily::Boxed, 24, 1e-5, "Reg1e_5"},
        {BenchmarkProblemFamily::Boxed, 24, 1e-4, "Reg1e_4"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 1e-6, "Reg1e_6"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 1e-5, "Reg1e_5"},
        {BenchmarkProblemFamily::FrictionIndex, 8, 1e-4, "Reg1e_4"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 1e-6, "Contacts16Reg1e_6"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 1e-5, "Contacts16Reg1e_5"},
        {BenchmarkProblemFamily::FrictionIndex, 16, 1e-4, "Contacts16Reg1e_4"},
    }};

std::string_view getNewtonWarmStartModeName(const NewtonWarmStartMode mode)
{
  switch (mode) {
    case NewtonWarmStartMode::None:
      return "None";
    case NewtonWarmStartMode::Pgs:
      return "Pgs";
    case NewtonWarmStartMode::GradientDescent:
      return "GradientDescent";
    case NewtonWarmStartMode::PgsThenGradient:
      return "PgsThenGradient";
  }

  return "Unknown";
}

std::string_view getRelaxationSweepKindName(const RelaxationSweepKind kind)
{
  switch (kind) {
    case RelaxationSweepKind::Under:
      return "Under";
    case RelaxationSweepKind::Plain:
      return "Plain";
    case RelaxationSweepKind::Over:
      return "Over";
  }

  return "Unknown";
}

bool usesPgsWarmStart(const NewtonWarmStartMode mode)
{
  return mode == NewtonWarmStartMode::Pgs
         || mode == NewtonWarmStartMode::PgsThenGradient;
}

bool usesGradientWarmStart(const NewtonWarmStartMode mode)
{
  return mode == NewtonWarmStartMode::GradientDescent
         || mode == NewtonWarmStartMode::PgsThenGradient;
}

enum class MildIllConditionedBenchmarkCase
{
  Standard32,
  Boxed16,
  FrictionIndex8,
  CoupledFrictionIndex6,
  CoupledFrictionIndex8,
  CoupledFrictionIndex12,
  CoupledFrictionIndex16,
  CoupledFrictionIndex24,
  CoupledFrictionIndex32,
  CoupledFrictionIndex48,
  CoupledFrictionIndex64,
  CoupledFrictionIndex96,
  StrongCoupledFrictionIndex6,
  StrongCoupledFrictionIndex8,
  StrongCoupledFrictionIndex12,
  StrongCoupledFrictionIndex16,
  StrongCoupledFrictionIndex24,
  StrongCoupledFrictionIndex32,
  StrongCoupledFrictionIndex48,
  StrongCoupledFrictionIndex64,
  StrongCoupledFrictionIndex96,
  StrongerCoupledFrictionIndex6,
  StrongerCoupledFrictionIndex8,
  StrongerCoupledFrictionIndex12,
  StrongerCoupledFrictionIndex16,
  StrongerCoupledFrictionIndex24,
  StrongerCoupledFrictionIndex32,
  StrongerCoupledFrictionIndex48,
  StrongerCoupledFrictionIndex64,
  StrongerCoupledFrictionIndex96,
  ExtremeCoupledFrictionIndex6,
  ExtremeCoupledFrictionIndex8,
  ExtremeCoupledFrictionIndex12,
  ExtremeCoupledFrictionIndex16,
  ExtremeCoupledFrictionIndex24,
  ExtremeCoupledFrictionIndex32,
  ExtremeCoupledFrictionIndex48,
  ExtremeCoupledFrictionIndex64,
  ExtremeCoupledFrictionIndex96,
  ExtremeCoupledFrictionIndex128,
  ExtremeCoupledFrictionIndex192,
  ExtremeCoupledFrictionIndex256
};

enum class NearSingularBenchmarkCase
{
  Standard8,
  Boxed8,
  CoupledFrictionIndex3,
  CoupledFrictionIndex6,
  CoupledFrictionIndex9,
  CoupledFrictionIndex12,
  CoupledFrictionIndex16,
  CoupledFrictionIndex24,
  CoupledFrictionIndex32,
  CoupledFrictionIndex48,
  CoupledFrictionIndex64,
  CoupledFrictionIndex96,
  CoupledFrictionIndex128,
  CoupledFrictionIndex192,
  CoupledFrictionIndex256
};

enum class SingularDegenerateBenchmarkCase
{
  Standard16,
  Boxed16,
  CoupledFrictionIndex6,
  Standard32,
  Boxed32,
  CoupledFrictionIndex8,
  Standard64,
  Boxed64,
  CoupledFrictionIndex12,
  Standard128,
  Boxed128,
  CoupledFrictionIndex16,
  CoupledFrictionIndex24,
  CoupledFrictionIndex32,
  CoupledFrictionIndex48,
  CoupledFrictionIndex64,
  CoupledFrictionIndex96,
  CoupledFrictionIndex128,
  CoupledFrictionIndex192,
  CoupledFrictionIndex256
};

enum class LargerActiveSetTransitionBenchmarkCase
{
  Standard32,
  Boxed32,
  CoupledFrictionIndex8,
  Standard64,
  Boxed64,
  CoupledFrictionIndex12,
  Standard128,
  Boxed128,
  CoupledFrictionIndex16,
  CoupledFrictionIndex24,
  CoupledFrictionIndex32,
  CoupledFrictionIndex48,
  CoupledFrictionIndex64,
  CoupledFrictionIndex96,
  CoupledFrictionIndex128,
  CoupledFrictionIndex192,
  CoupledFrictionIndex256
};

std::string_view getProblemFamilyName(BenchmarkProblemFamily family)
{
  switch (family) {
    case BenchmarkProblemFamily::Standard:
      return "Standard";
    case BenchmarkProblemFamily::Boxed:
      return "Boxed";
    case BenchmarkProblemFamily::FrictionIndex:
      return "FrictionIndex";
  }

  return "Unknown";
}

LcpProblem MakeBenchmarkProblem(BenchmarkProblemFamily family, int size)
{
  switch (family) {
    case BenchmarkProblemFamily::Standard:
      return MakeStandardSpdProblem(
          size, 10'001u + static_cast<unsigned>(size));
    case BenchmarkProblemFamily::Boxed:
      return MakeBoxedActiveBoundsProblem(
          size, 20'001u + static_cast<unsigned>(size));
    case BenchmarkProblemFamily::FrictionIndex:
      return MakeFrictionIndexProblem(
          size, 30'001u + static_cast<unsigned>(size));
  }

  return MakeStandardSpdProblem(size, 10'001u + static_cast<unsigned>(size));
}

std::vector<int> MakeBlockPartition(
    const LcpProblem& problem, const BlockPartitionSweepCase testCase)
{
  const auto problemSize = static_cast<int>(problem.b.size());
  switch (testCase.profile) {
    case BlockPartitionProfile::FullBlock:
      return {problemSize};
    case BlockPartitionProfile::UniformThree: {
      std::vector<int> blocks;
      blocks.reserve(static_cast<std::size_t>((problemSize + 2) / 3));
      int remaining = problemSize;
      while (remaining > 0) {
        const int blockSize = std::min(3, remaining);
        blocks.push_back(blockSize);
        remaining -= blockSize;
      }
      return blocks;
    }
    case BlockPartitionProfile::AutoFindex:
      return {};
    case BlockPartitionProfile::ContactBlocks:
      return std::vector<int>(static_cast<std::size_t>(testCase.problemArg), 3);
  }

  return {problemSize};
}

LcpProblem MakeActiveSetTransitionBenchmarkProblem(
    BenchmarkProblemFamily family)
{
  switch (family) {
    case BenchmarkProblemFamily::Standard:
      return MakeStandardActiveSetTransitionProblem();
    case BenchmarkProblemFamily::Boxed:
      return MakeBoxedActiveSetTransitionProblem();
    case BenchmarkProblemFamily::FrictionIndex:
      return MakeFrictionIndexActiveSetTransitionProblem();
  }

  return MakeStandardActiveSetTransitionProblem();
}

std::string_view getLargerActiveSetTransitionCaseName(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
      return "Standard32";
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
      return "Boxed32";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
      return "CoupledFrictionIndex8";
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
      return "Standard64";
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
      return "Boxed64";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
      return "CoupledFrictionIndex12";
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
      return "Standard128";
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return "Boxed128";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return "CoupledFrictionIndex16";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return "CoupledFrictionIndex24";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return "CoupledFrictionIndex32";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48:
      return "CoupledFrictionIndex48";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64:
      return "CoupledFrictionIndex64";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96:
      return "CoupledFrictionIndex96";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128:
      return "CoupledFrictionIndex128";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192:
      return "CoupledFrictionIndex192";
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256:
      return "CoupledFrictionIndex256";
  }

  return "Unknown";
}

bool isLargerActiveSetTransitionFrictionIndexCase(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  return testCase
             == LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex12
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex16
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex24
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex32
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex48
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex64
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex96
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex128
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex192
         || testCase
                == LargerActiveSetTransitionBenchmarkCase::
                    CoupledFrictionIndex256;
}

int getLargerActiveSetTransitionContactCount(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
      return 8;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
      return 12;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return 16;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return 24;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return 32;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48:
      return 48;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64:
      return 64;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96:
      return 96;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128:
      return 128;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192:
      return 192;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256:
      return 256;
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return 0;
  }

  return 0;
}

unsigned getLargerActiveSetTransitionSeedBase(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
      return 21'032u;
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
      return 22'032u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
      return 23'008u;
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
      return 21'064u;
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
      return 22'064u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
      return 23'012u;
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
      return 21'128u;
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return 22'128u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return 23'016u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return 23'024u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return 23'032u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48:
      return 23'048u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64:
      return 23'064u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96:
      return 23'096u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128:
      return 23'128u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192:
      return 23'192u;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256:
      return 23'256u;
  }

  return 21'032u;
}

LcpProblem MakeLargerActiveSetTransitionBenchmarkProblem(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const unsigned seedOffset = 0)
{
  const unsigned seed
      = getLargerActiveSetTransitionSeedBase(testCase) + seedOffset;
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
      return MakeStandardActiveSetTransitionProblem(32, seed);
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
      return MakeBoxedActiveSetTransitionProblem(32, seed);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
      return MakeFrictionIndexActiveSetTransitionProblem(8, seed);
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
      return MakeStandardActiveSetTransitionProblem(64, seed);
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
      return MakeBoxedActiveSetTransitionProblem(64, seed);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
      return MakeFrictionIndexActiveSetTransitionProblem(12, seed);
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
      return MakeStandardActiveSetTransitionProblem(128, seed);
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return MakeBoxedActiveSetTransitionProblem(128, seed);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return MakeFrictionIndexActiveSetTransitionProblem(16, seed);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return MakeFrictionIndexActiveSetTransitionProblem(24, seed, 2.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return MakeFrictionIndexActiveSetTransitionProblem(32, seed, 4.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48:
      return MakeFrictionIndexActiveSetTransitionProblem(48, seed, 8.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64:
      return MakeFrictionIndexActiveSetTransitionProblem(64, seed, 16.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96:
      return MakeFrictionIndexActiveSetTransitionProblem(96, seed, 32.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128:
      return MakeFrictionIndexActiveSetTransitionProblem(128, seed, 32.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192:
      return MakeFrictionIndexActiveSetTransitionProblem(192, seed, 32.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256:
      return MakeFrictionIndexActiveSetTransitionProblem(256, seed, 32.0);
  }

  return MakeStandardActiveSetTransitionProblem(32, seed);
}

double getLargerActiveSetTransitionCouplingScale(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return 2.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return 4.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48:
      return 8.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64:
      return 16.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96:
      return 32.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128:
      return 32.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192:
      return 32.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256:
      return 32.0;
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return 1.0;
  }

  return 1.0;
}

std::string_view getMildIllConditionedCaseName(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::Standard32:
      return "Standard32";
    case MildIllConditionedBenchmarkCase::Boxed16:
      return "Boxed16";
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
      return "FrictionIndex8";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
      return "CoupledFrictionIndex6";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
      return "CoupledFrictionIndex8";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
      return "CoupledFrictionIndex12";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
      return "CoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
      return "CoupledFrictionIndex24";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
      return "CoupledFrictionIndex32";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
      return "CoupledFrictionIndex48";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
      return "CoupledFrictionIndex64";
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
      return "CoupledFrictionIndex96";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
      return "StrongCoupledFrictionIndex6";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
      return "StrongCoupledFrictionIndex8";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
      return "StrongCoupledFrictionIndex12";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      return "StrongCoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      return "StrongCoupledFrictionIndex24";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
      return "StrongCoupledFrictionIndex32";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
      return "StrongCoupledFrictionIndex48";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
      return "StrongCoupledFrictionIndex64";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
      return "StrongCoupledFrictionIndex96";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
      return "StrongerCoupledFrictionIndex6";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
      return "StrongerCoupledFrictionIndex8";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
      return "StrongerCoupledFrictionIndex12";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      return "StrongerCoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return "StrongerCoupledFrictionIndex24";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
      return "StrongerCoupledFrictionIndex32";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
      return "StrongerCoupledFrictionIndex48";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
      return "StrongerCoupledFrictionIndex64";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      return "StrongerCoupledFrictionIndex96";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
      return "ExtremeCoupledFrictionIndex6";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
      return "ExtremeCoupledFrictionIndex8";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
      return "ExtremeCoupledFrictionIndex12";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
      return "ExtremeCoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
      return "ExtremeCoupledFrictionIndex24";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
      return "ExtremeCoupledFrictionIndex32";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
      return "ExtremeCoupledFrictionIndex48";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
      return "ExtremeCoupledFrictionIndex64";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
      return "ExtremeCoupledFrictionIndex96";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
      return "ExtremeCoupledFrictionIndex128";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
      return "ExtremeCoupledFrictionIndex192";
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
      return "ExtremeCoupledFrictionIndex256";
  }

  return "Unknown";
}

template <std::size_t N>
bool SolverNameIn(
    const dart::test::LcpSolverManifestEntry& solver,
    const std::array<std::string_view, N>& names)
{
  return std::find(names.begin(), names.end(), solver.name) != names.end();
}

const dart::test::LcpSolverManifestEntry* FindSolverManifestEntry(
    const std::string_view solverName)
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (solver.name == solverName) {
      return &solver;
    }
  }

  return nullptr;
}

bool SolverSupportsConcreteProblem(
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const LcpProblem& problem)
{
  const auto solver = solverEntry.create();
  return solver != nullptr && solver->supportsProblem(problem);
}

bool SolverSupportsConcreteProblemBatch(
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const std::vector<LcpProblem>& problems)
{
  const auto solver = solverEntry.create();
  if (solver == nullptr || problems.empty()) {
    return false;
  }

  return std::ranges::all_of(problems, [&](const LcpProblem& problem) {
    return solver->supportsProblem(problem);
  });
}

bool isMildIllConditionedFrictionIndexCase(
    const MildIllConditionedBenchmarkCase testCase)
{
  return testCase == MildIllConditionedBenchmarkCase::FrictionIndex8
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex6
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex8
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex12
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex16
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex24
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex32
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex48
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex64
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex128
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex192
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex256;
}

int getMildIllConditionedContactCount(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
      return 8;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
      return 6;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
      return 12;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
      return 16;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
      return 24;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
      return 32;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
      return 48;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
      return 64;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      return 96;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
      return 32;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
      return 48;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
      return 64;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
      return 96;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
      return 128;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
      return 192;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
      return 256;
    case MildIllConditionedBenchmarkCase::Standard32:
    case MildIllConditionedBenchmarkCase::Boxed16:
      return 0;
  }

  return 0;
}

bool isMildIllConditionedCoupledFrictionIndexCase(
    const MildIllConditionedBenchmarkCase testCase)
{
  return testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex6
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex8
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex12
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex16
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex24
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex32
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex48
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex64
         || testCase == MildIllConditionedBenchmarkCase::CoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6
         || testCase
                == MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex12
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex32
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex48
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex64
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex96
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex128
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex192
         || testCase
                == MildIllConditionedBenchmarkCase::
                    ExtremeCoupledFrictionIndex256;
}

double getMildIllConditionedCouplingScale(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
      return 4.0;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      return 8.0;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
      return 16.0;
    case MildIllConditionedBenchmarkCase::Standard32:
    case MildIllConditionedBenchmarkCase::Boxed16:
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
      return 1.0;
  }

  return 1.0;
}

LcpProblem MakeMildIllConditionedBenchmarkProblem(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::Standard32:
      return MakeMildIllConditionedStandardProblem(32, 24'032u);
    case MildIllConditionedBenchmarkCase::Boxed16:
      return MakeMildIllConditionedBoxedProblem(16, 25'016u);
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
      return MakeMildIllConditionedFrictionIndexProblem(8, 26'008u, false);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
      return MakeMildIllConditionedFrictionIndexProblem(6, 27'006u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
      return MakeMildIllConditionedFrictionIndexProblem(8, 27'008u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
      return MakeMildIllConditionedFrictionIndexProblem(12, 27'012u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(16, 27'016u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(24, 27'024u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
      return MakeMildIllConditionedFrictionIndexProblem(32, 27'032u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
      return MakeMildIllConditionedFrictionIndexProblem(48, 27'048u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
      return MakeMildIllConditionedFrictionIndexProblem(64, 27'064u, true);
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
      return MakeMildIllConditionedFrictionIndexProblem(96, 27'096u, true);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
      return MakeMildIllConditionedFrictionIndexProblem(6, 28'006u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
      return MakeMildIllConditionedFrictionIndexProblem(8, 28'008u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
      return MakeMildIllConditionedFrictionIndexProblem(12, 28'012u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(16, 28'016u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(24, 28'024u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
      return MakeMildIllConditionedFrictionIndexProblem(32, 28'032u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
      return MakeMildIllConditionedFrictionIndexProblem(48, 28'048u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
      return MakeMildIllConditionedFrictionIndexProblem(64, 28'064u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
      return MakeMildIllConditionedFrictionIndexProblem(96, 28'096u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
      return MakeMildIllConditionedFrictionIndexProblem(6, 29'006u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
      return MakeMildIllConditionedFrictionIndexProblem(8, 29'008u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
      return MakeMildIllConditionedFrictionIndexProblem(12, 29'012u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(16, 29'016u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(24, 29'024u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
      return MakeMildIllConditionedFrictionIndexProblem(32, 29'032u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
      return MakeMildIllConditionedFrictionIndexProblem(48, 29'048u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
      return MakeMildIllConditionedFrictionIndexProblem(64, 29'064u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      return MakeMildIllConditionedFrictionIndexProblem(96, 29'096u, true, 8.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
      return MakeMildIllConditionedFrictionIndexProblem(6, 30'006u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
      return MakeMildIllConditionedFrictionIndexProblem(8, 30'008u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
      return MakeMildIllConditionedFrictionIndexProblem(
          12, 30'012u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(
          16, 30'016u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(
          24, 30'024u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
      return MakeMildIllConditionedFrictionIndexProblem(
          32, 30'032u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
      return MakeMildIllConditionedFrictionIndexProblem(
          48, 30'048u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
      return MakeMildIllConditionedFrictionIndexProblem(
          64, 30'064u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
      return MakeMildIllConditionedFrictionIndexProblem(
          96, 30'096u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
      return MakeMildIllConditionedFrictionIndexProblem(
          128, 30'128u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
      return MakeMildIllConditionedFrictionIndexProblem(
          192, 30'192u, true, 16.0);
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
      return MakeMildIllConditionedFrictionIndexProblem(
          256, 30'256u, true, 16.0);
  }

  return MakeMildIllConditionedStandardProblem(32, 24'032u);
}

std::string_view getNearSingularCaseName(
    const NearSingularBenchmarkCase testCase)
{
  switch (testCase) {
    case NearSingularBenchmarkCase::Standard8:
      return "Standard8";
    case NearSingularBenchmarkCase::Boxed8:
      return "Boxed8";
    case NearSingularBenchmarkCase::CoupledFrictionIndex3:
      return "CoupledFrictionIndex3";
    case NearSingularBenchmarkCase::CoupledFrictionIndex6:
      return "CoupledFrictionIndex6";
    case NearSingularBenchmarkCase::CoupledFrictionIndex9:
      return "CoupledFrictionIndex9";
    case NearSingularBenchmarkCase::CoupledFrictionIndex12:
      return "CoupledFrictionIndex12";
    case NearSingularBenchmarkCase::CoupledFrictionIndex16:
      return "CoupledFrictionIndex16";
    case NearSingularBenchmarkCase::CoupledFrictionIndex24:
      return "CoupledFrictionIndex24";
    case NearSingularBenchmarkCase::CoupledFrictionIndex32:
      return "CoupledFrictionIndex32";
    case NearSingularBenchmarkCase::CoupledFrictionIndex48:
      return "CoupledFrictionIndex48";
    case NearSingularBenchmarkCase::CoupledFrictionIndex64:
      return "CoupledFrictionIndex64";
    case NearSingularBenchmarkCase::CoupledFrictionIndex96:
      return "CoupledFrictionIndex96";
    case NearSingularBenchmarkCase::CoupledFrictionIndex128:
      return "CoupledFrictionIndex128";
    case NearSingularBenchmarkCase::CoupledFrictionIndex192:
      return "CoupledFrictionIndex192";
    case NearSingularBenchmarkCase::CoupledFrictionIndex256:
      return "CoupledFrictionIndex256";
  }

  return "Unknown";
}

bool isNearSingularFrictionIndexCase(const NearSingularBenchmarkCase testCase)
{
  return testCase == NearSingularBenchmarkCase::CoupledFrictionIndex3
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex6
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex9
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex12
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex16
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex24
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex32
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex48
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex64
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex96
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex128
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex192
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex256;
}

int getNearSingularContactCount(const NearSingularBenchmarkCase testCase)
{
  switch (testCase) {
    case NearSingularBenchmarkCase::CoupledFrictionIndex3:
      return 3;
    case NearSingularBenchmarkCase::CoupledFrictionIndex6:
      return 6;
    case NearSingularBenchmarkCase::CoupledFrictionIndex9:
      return 9;
    case NearSingularBenchmarkCase::CoupledFrictionIndex12:
      return 12;
    case NearSingularBenchmarkCase::CoupledFrictionIndex16:
      return 16;
    case NearSingularBenchmarkCase::CoupledFrictionIndex24:
      return 24;
    case NearSingularBenchmarkCase::CoupledFrictionIndex32:
      return 32;
    case NearSingularBenchmarkCase::CoupledFrictionIndex48:
      return 48;
    case NearSingularBenchmarkCase::CoupledFrictionIndex64:
      return 64;
    case NearSingularBenchmarkCase::CoupledFrictionIndex96:
      return 96;
    case NearSingularBenchmarkCase::CoupledFrictionIndex128:
      return 128;
    case NearSingularBenchmarkCase::CoupledFrictionIndex192:
      return 192;
    case NearSingularBenchmarkCase::CoupledFrictionIndex256:
      return 256;
    case NearSingularBenchmarkCase::Standard8:
    case NearSingularBenchmarkCase::Boxed8:
      return 0;
  }

  return 0;
}

LcpProblem MakeNearSingularBenchmarkProblem(
    const NearSingularBenchmarkCase testCase)
{
  switch (testCase) {
    case NearSingularBenchmarkCase::Standard8:
      return MakeNearSingularStandardProblem(8, 14'008u);
    case NearSingularBenchmarkCase::Boxed8:
      return MakeNearSingularBoxedProblem(8, 15'008u);
    case NearSingularBenchmarkCase::CoupledFrictionIndex3:
      return MakeNearSingularFrictionIndexProblem(3, 16'003u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex6:
      return MakeNearSingularFrictionIndexProblem(6, 16'006u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex9:
      return MakeNearSingularFrictionIndexProblem(9, 16'009u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex12:
      return MakeNearSingularFrictionIndexProblem(12, 16'012u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex16:
      return MakeNearSingularFrictionIndexProblem(16, 16'016u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex24:
      return MakeNearSingularFrictionIndexProblem(24, 16'024u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex32:
      return MakeNearSingularFrictionIndexProblem(32, 16'032u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex48:
      return MakeNearSingularFrictionIndexProblem(48, 16'048u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex64:
      return MakeNearSingularFrictionIndexProblem(64, 16'064u, true);
    case NearSingularBenchmarkCase::CoupledFrictionIndex96:
      return MakeNearSingularFrictionIndexProblem(96, 16'096u, true, 0.08, 1e3);
    case NearSingularBenchmarkCase::CoupledFrictionIndex128:
      return MakeNearSingularFrictionIndexProblem(
          128, 16'128u, true, 0.08, 1e3);
    case NearSingularBenchmarkCase::CoupledFrictionIndex192:
      return MakeNearSingularFrictionIndexProblem(
          192, 16'192u, true, 0.08, 1e3);
    case NearSingularBenchmarkCase::CoupledFrictionIndex256:
      return MakeNearSingularFrictionIndexProblem(
          256, 16'256u, true, 0.08, 1e3);
  }

  return MakeNearSingularStandardProblem(8, 14'008u);
}

std::vector<LcpProblem> MakeNearSingularBatchProblems(
    const NearSingularBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  for (const int i : std::views::iota(0, batchSize)) {
    const unsigned variant = static_cast<unsigned>(i);
    switch (testCase) {
      case NearSingularBenchmarkCase::Standard8:
        problems.push_back(
            MakeNearSingularStandardProblem(8, 14'008u + variant));
        break;
      case NearSingularBenchmarkCase::Boxed8:
        problems.push_back(MakeNearSingularBoxedProblem(8, 15'008u + variant));
        break;
      case NearSingularBenchmarkCase::CoupledFrictionIndex3:
      case NearSingularBenchmarkCase::CoupledFrictionIndex6:
      case NearSingularBenchmarkCase::CoupledFrictionIndex9:
      case NearSingularBenchmarkCase::CoupledFrictionIndex12:
      case NearSingularBenchmarkCase::CoupledFrictionIndex16:
      case NearSingularBenchmarkCase::CoupledFrictionIndex24:
      case NearSingularBenchmarkCase::CoupledFrictionIndex32:
      case NearSingularBenchmarkCase::CoupledFrictionIndex48:
      case NearSingularBenchmarkCase::CoupledFrictionIndex64:
      case NearSingularBenchmarkCase::CoupledFrictionIndex96:
      case NearSingularBenchmarkCase::CoupledFrictionIndex128:
      case NearSingularBenchmarkCase::CoupledFrictionIndex192:
      case NearSingularBenchmarkCase::CoupledFrictionIndex256: {
        const int contactCount = getNearSingularContactCount(testCase);
        const unsigned seedBase = 16'000u + static_cast<unsigned>(contactCount);
        // Keep the 64-contact batch on the same contract-verified packet as
        // the matching single-problem benchmark.
        const unsigned problemSeed
            = contactCount == 64 ? seedBase : seedBase + variant;
        const double normalSlope = contactCount >= 96 ? 0.08 : 0.15;
        const double nearSingularScale = contactCount >= 96 ? 1e3 : 1e4;
        problems.push_back(MakeNearSingularFrictionIndexProblem(
            contactCount, problemSeed, true, normalSlope, nearSingularScale));
        break;
      }
    }
  }

  return problems;
}

LcpProblem MakeSingularDegenerateStandardProblem(
    const int n, const int variant = 0)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    const int shifted = i + variant;
    if (shifted % 4 == 0) {
      w[i] = 0.25 + 0.02 * static_cast<double>(shifted + 1);
    } else {
      A(i, i) = 1.0 + 0.15 * static_cast<double>(shifted % 7);
      x[i] = 0.08 + 0.03 * static_cast<double>((shifted % 5) + 1);
    }
  }

  Eigen::VectorXd b = A * x - w;
  const double inf = std::numeric_limits<double>::infinity();
  return LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, inf),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeSingularDegenerateBoxedProblem(
    const int n, const int variant = 0)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  const double inf = std::numeric_limits<double>::infinity();

  for (int i = 0; i < n; ++i) {
    const int shifted = i + variant;
    const int mode = shifted % 4;
    if (mode == 0) {
      lo[i] = -0.45;
      hi[i] = 0.55;
      x[i] = lo[i];
      w[i] = 0.2 + 0.01 * static_cast<double>(shifted + 1);
    } else if (mode == 1) {
      lo[i] = -0.35;
      hi[i] = 0.4;
      x[i] = hi[i];
      w[i] = -0.18 - 0.01 * static_cast<double>(shifted + 1);
    } else if (mode == 2) {
      lo[i] = -0.7;
      hi[i] = 0.75;
      A(i, i) = 1.2 + 0.1 * static_cast<double>(shifted % 5);
      x[i] = -0.1 + 0.04 * static_cast<double>(shifted % 4);
    } else {
      lo[i] = -0.25;
      hi[i] = inf;
      A(i, i) = 1.1 + 0.12 * static_cast<double>(shifted % 6);
      x[i] = 0.15 + 0.03 * static_cast<double>(shifted % 5);
    }
  }

  Eigen::VectorXd b = A * x - w;
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeSingularDegenerateFrictionIndexProblem(
    const int numContacts, const unsigned couplingSeed = 31'006u)
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
      const double value
          = magnitude * BenchmarkSignedUnitValue(r, c, couplingSeed);
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

std::string_view getSingularDegenerateCaseName(
    const SingularDegenerateBenchmarkCase testCase)
{
  switch (testCase) {
    case SingularDegenerateBenchmarkCase::Standard16:
      return "Standard16";
    case SingularDegenerateBenchmarkCase::Boxed16:
      return "Boxed16";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex6:
      return "CoupledFrictionIndex6";
    case SingularDegenerateBenchmarkCase::Standard32:
      return "Standard32";
    case SingularDegenerateBenchmarkCase::Boxed32:
      return "Boxed32";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex8:
      return "CoupledFrictionIndex8";
    case SingularDegenerateBenchmarkCase::Standard64:
      return "Standard64";
    case SingularDegenerateBenchmarkCase::Boxed64:
      return "Boxed64";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex12:
      return "CoupledFrictionIndex12";
    case SingularDegenerateBenchmarkCase::Standard128:
      return "Standard128";
    case SingularDegenerateBenchmarkCase::Boxed128:
      return "Boxed128";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex16:
      return "CoupledFrictionIndex16";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex24:
      return "CoupledFrictionIndex24";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex32:
      return "CoupledFrictionIndex32";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex48:
      return "CoupledFrictionIndex48";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex64:
      return "CoupledFrictionIndex64";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex96:
      return "CoupledFrictionIndex96";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex128:
      return "CoupledFrictionIndex128";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex192:
      return "CoupledFrictionIndex192";
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex256:
      return "CoupledFrictionIndex256";
  }

  return "Unknown";
}

bool isSingularDegenerateFrictionIndexCase(
    const SingularDegenerateBenchmarkCase testCase)
{
  return testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex6
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex8
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex12
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex16
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex24
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex32
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex48
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex64
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex96
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex128
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex192
         || testCase
                == SingularDegenerateBenchmarkCase::CoupledFrictionIndex256;
}

int getSingularDegenerateContactCount(
    const SingularDegenerateBenchmarkCase testCase)
{
  switch (testCase) {
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex6:
      return 6;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex8:
      return 8;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex12:
      return 12;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex16:
      return 16;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex24:
      return 24;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex32:
      return 32;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex48:
      return 48;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex64:
      return 64;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex96:
      return 96;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex128:
      return 128;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex192:
      return 192;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex256:
      return 256;
    case SingularDegenerateBenchmarkCase::Standard16:
    case SingularDegenerateBenchmarkCase::Boxed16:
    case SingularDegenerateBenchmarkCase::Standard32:
    case SingularDegenerateBenchmarkCase::Boxed32:
    case SingularDegenerateBenchmarkCase::Standard64:
    case SingularDegenerateBenchmarkCase::Boxed64:
    case SingularDegenerateBenchmarkCase::Standard128:
    case SingularDegenerateBenchmarkCase::Boxed128:
      return 0;
  }

  return 0;
}

LcpProblem MakeSingularDegenerateBenchmarkProblem(
    const SingularDegenerateBenchmarkCase testCase)
{
  switch (testCase) {
    case SingularDegenerateBenchmarkCase::Standard16:
      return MakeSingularDegenerateStandardProblem(16);
    case SingularDegenerateBenchmarkCase::Boxed16:
      return MakeSingularDegenerateBoxedProblem(16);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex6:
      return MakeSingularDegenerateFrictionIndexProblem(6);
    case SingularDegenerateBenchmarkCase::Standard32:
      return MakeSingularDegenerateStandardProblem(32);
    case SingularDegenerateBenchmarkCase::Boxed32:
      return MakeSingularDegenerateBoxedProblem(32);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex8:
      return MakeSingularDegenerateFrictionIndexProblem(8);
    case SingularDegenerateBenchmarkCase::Standard64:
      return MakeSingularDegenerateStandardProblem(64);
    case SingularDegenerateBenchmarkCase::Boxed64:
      return MakeSingularDegenerateBoxedProblem(64);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex12:
      return MakeSingularDegenerateFrictionIndexProblem(12);
    case SingularDegenerateBenchmarkCase::Standard128:
      return MakeSingularDegenerateStandardProblem(128);
    case SingularDegenerateBenchmarkCase::Boxed128:
      return MakeSingularDegenerateBoxedProblem(128);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex16:
      return MakeSingularDegenerateFrictionIndexProblem(16);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex24:
      return MakeSingularDegenerateFrictionIndexProblem(24);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex32:
      return MakeSingularDegenerateFrictionIndexProblem(32);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex48:
      return MakeSingularDegenerateFrictionIndexProblem(48);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex64:
      return MakeSingularDegenerateFrictionIndexProblem(64);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex96:
      return MakeSingularDegenerateFrictionIndexProblem(96);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex128:
      return MakeSingularDegenerateFrictionIndexProblem(128);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex192:
      return MakeSingularDegenerateFrictionIndexProblem(192);
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex256:
      return MakeSingularDegenerateFrictionIndexProblem(256);
  }

  return MakeSingularDegenerateStandardProblem(16);
}

std::vector<LcpProblem> MakeSingularDegenerateFrictionIndexBatchProblems(
    const SingularDegenerateBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  const int contactCount = getSingularDegenerateContactCount(testCase);
  const unsigned seedBase = 31'000u + static_cast<unsigned>(contactCount);

  for (const int i : std::views::iota(0, batchSize)) {
    problems.push_back(MakeSingularDegenerateFrictionIndexProblem(
        contactCount, seedBase + static_cast<unsigned>(i)));
  }

  return problems;
}

std::vector<LcpProblem> MakeSingularDegenerateStandardBoxedBatchProblems(
    const SingularDegenerateBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  for (const int i : std::views::iota(0, batchSize)) {
    switch (testCase) {
      case SingularDegenerateBenchmarkCase::Standard16:
        problems.push_back(MakeSingularDegenerateStandardProblem(16, i));
        break;
      case SingularDegenerateBenchmarkCase::Boxed16:
        problems.push_back(MakeSingularDegenerateBoxedProblem(16, i));
        break;
      case SingularDegenerateBenchmarkCase::Standard32:
        problems.push_back(MakeSingularDegenerateStandardProblem(32, i));
        break;
      case SingularDegenerateBenchmarkCase::Boxed32:
        problems.push_back(MakeSingularDegenerateBoxedProblem(32, i));
        break;
      case SingularDegenerateBenchmarkCase::Standard64:
        problems.push_back(MakeSingularDegenerateStandardProblem(64, i));
        break;
      case SingularDegenerateBenchmarkCase::Boxed64:
        problems.push_back(MakeSingularDegenerateBoxedProblem(64, i));
        break;
      case SingularDegenerateBenchmarkCase::Standard128:
        problems.push_back(MakeSingularDegenerateStandardProblem(128, i));
        break;
      case SingularDegenerateBenchmarkCase::Boxed128:
        problems.push_back(MakeSingularDegenerateBoxedProblem(128, i));
        break;
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex6:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex8:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex12:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex16:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex24:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex32:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex48:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex64:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex96:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex128:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex192:
      case SingularDegenerateBenchmarkCase::CoupledFrictionIndex256:
        problems.push_back(MakeSingularDegenerateStandardProblem(16, i));
        break;
    }
  }

  return problems;
}

int getBenchmarkMaxIterations(const dart::test::LcpSolverManifestEntry& solver)
{
  if (solver.name == "Direct") {
    return 1;
  }

  if (solver.name == "InteriorPoint" || solver.name == "MinimumMapNewton"
      || solver.name == "FischerBurmeisterNewton"
      || solver.name == "PenalizedFischerBurmeisterNewton"
      || solver.name == "BoxedSemiSmoothNewton") {
    return 50;
  }

  if (solver.name == "Sap") {
    return 5000;
  }

  if (solver.name == "Dantzig" || solver.name == "Baraff"
      || solver.name == "Lemke" || solver.name == "Pgs"
      || solver.name == "SymmetricPsor" || solver.name == "RedBlackGaussSeidel"
      || solver.name == "ShockPropagation" || solver.name == "Staggering") {
    return 100;
  }

  return 200;
}

struct SolverBenchmarkOptions
{
  LcpOptions options;
  dart::math::NncgSolver::Parameters nncgParams;
  dart::math::SubspaceMinimizationSolver::Parameters subspaceParams;
  dart::math::PenalizedFischerBurmeisterNewtonSolver::Parameters
      penalizedFischerBurmeisterParams;
  dart::math::BoxedSemiSmoothNewtonSolver::Parameters boxedSsnParams;
  dart::math::SapSolver::Parameters sapParams;
  dart::math::ShockPropagationSolver::Parameters shockPropagationParams;
  bool hasNncgParams{false};
  bool hasBoxedSsnParams{false};
  bool hasSapParams{false};
  bool hasShockPropagationParams{false};
};

struct NewtonWarmStartBenchmarkOptions
{
  LcpOptions options;
  dart::math::MinimumMapNewtonSolver::Parameters minimumMapParams;
  dart::math::FischerBurmeisterNewtonSolver::Parameters fischerBurmeisterParams;
  dart::math::PenalizedFischerBurmeisterNewtonSolver::Parameters
      penalizedFischerBurmeisterParams;
};

struct BatchBenchmarkCounters
{
  int problemCount{0};
  double totalIterations{0.0};
  double maxResidual{0.0};
  double maxComplementarity{0.0};
  double maxBoundViolation{0.0};
  bool allContractsOk{true};
};

void ConfigureShockPropagationParameters(
    const LcpProblem& problem,
    dart::math::ShockPropagationSolver::Parameters& params)
{
  params.blockSizes.clear();
  params.layers.clear();

  if ((problem.findex.array() >= 0).any()) {
    // Friction-index contact systems are not required to store each contact's
    // normal and tangent rows contiguously. Let ShockPropagationSolver group
    // each contact block from findex so world-contact snapshots and synthetic
    // contact packets use the same semantic block partition.
    return;
  }

  constexpr int blockSize = 3;
  int remaining = static_cast<int>(problem.b.size());
  while (remaining > 0) {
    const int size = std::min(blockSize, remaining);
    params.blockSizes.push_back(size);
    remaining -= size;
  }

  params.layers.reserve(params.blockSizes.size());
  for (const auto i :
       std::views::iota(std::ptrdiff_t{0}, std::ssize(params.blockSizes))) {
    params.layers.push_back({static_cast<int>(i)});
  }
}

void ConfigureShockPropagationLayerSweepParameters(
    const LcpProblem& problem,
    const ShockPropagationLayerProfile profile,
    dart::math::ShockPropagationSolver::Parameters& params)
{
  params.blockSizes.clear();
  params.layers.clear();

  constexpr int blockSize = 3;
  int remaining = static_cast<int>(problem.b.size());
  while (remaining > 0) {
    const int size = std::min(blockSize, remaining);
    params.blockSizes.push_back(size);
    remaining -= size;
  }

  const int blockCount = static_cast<int>(std::ssize(params.blockSizes));
  switch (profile) {
    case ShockPropagationLayerProfile::SingleLayer: {
      auto blockIndices = std::views::iota(0, blockCount);
      params.layers.emplace_back(blockIndices.begin(), blockIndices.end());
      break;
    }
    case ShockPropagationLayerProfile::TwoLayers: {
      const int split = (blockCount + 1) / 2;
      auto firstLayer = std::views::iota(0, split);
      auto secondLayer = std::views::iota(split, blockCount);
      params.layers.emplace_back(firstLayer.begin(), firstLayer.end());
      params.layers.emplace_back(secondLayer.begin(), secondLayer.end());
      break;
    }
    case ShockPropagationLayerProfile::SerialLayers:
      params.layers.reserve(blockCount);
      for (const int blockIndex : std::views::iota(0, blockCount)) {
        params.layers.push_back({blockIndex});
      }
      break;
  }
}

LcpProblem MakeStandardSpdSweepProblem(
    const StandardSpdProblemKind kind,
    const int problemSize,
    const unsigned seed)
{
  switch (kind) {
    case StandardSpdProblemKind::DenseSpd:
      return MakeStandardSpdProblem(problemSize, seed);
    case StandardSpdProblemKind::BandedSpd:
      return MakeStandardBandedSpdProblem(problemSize, seed);
    case StandardSpdProblemKind::MildIllConditioned:
      return MakeMildIllConditionedStandardProblem(problemSize, seed);
    case StandardSpdProblemKind::NearSingular:
      return MakeNearSingularStandardProblem(problemSize, seed);
  }

  return MakeStandardSpdProblem(problemSize, seed);
}

void ConfigureSolverBenchmarkOptions(
    SolverBenchmarkOptions& storage,
    const dart::test::LcpSolverManifestEntry& solver,
    const LcpProblem& problem)
{
  storage.options = MakeBenchmarkOptions(getBenchmarkMaxIterations(solver));

  if (solver.name == "NNCG") {
    storage.nncgParams.pgsIterations = 2;
    storage.nncgParams.restartInterval = 10;
    storage.nncgParams.restartThreshold = 1.0;
    storage.options.customOptions = &storage.nncgParams;
    storage.hasNncgParams = true;
  } else if (solver.name == "SubspaceMinimization") {
    storage.subspaceParams.pgsIterations = 5;
    storage.options.customOptions = &storage.subspaceParams;
  } else if (solver.name == "PenalizedFischerBurmeisterNewton") {
    storage.penalizedFischerBurmeisterParams.lambda = 1.0;
    storage.options.customOptions = &storage.penalizedFischerBurmeisterParams;
  } else if (solver.name == "BoxedSemiSmoothNewton") {
    const bool hasFrictionIndex = (problem.findex.array() >= 0).any();
    const bool hasBoxBounds = problem.hi.array().isFinite().any()
                              || (problem.lo.array() < 0.0).any();
    if (hasFrictionIndex || hasBoxBounds) {
      storage.boxedSsnParams.maxPgsWarmStartIterations = 5;
      storage.options.customOptions = &storage.boxedSsnParams;
      storage.hasBoxedSsnParams = true;
    }
  } else if (solver.name == "Sap") {
    storage.options.absoluteTolerance = 1e-5;
    storage.options.relativeTolerance = 1e-3;
    const bool hasFrictionIndex = (problem.findex.array() >= 0).any();
    storage.options.complementarityTolerance = hasFrictionIndex ? 2e-3 : 1e-3;
    const bool hasBoxBounds = problem.hi.array().isFinite().any()
                              || (problem.lo.array() < 0.0).any();
    if (!hasFrictionIndex && hasBoxBounds) {
      storage.sapParams.regularization = 1e-4;
    } else {
      storage.sapParams.regularization = 1e-6;
    }
    storage.sapParams.maxLineSearchIterations = 32;
    storage.options.customOptions = &storage.sapParams;
    storage.hasSapParams = true;
  } else if (solver.name == "ShockPropagation") {
    ConfigureShockPropagationParameters(
        problem, storage.shockPropagationParams);
    storage.options.customOptions = &storage.shockPropagationParams;
    storage.hasShockPropagationParams = true;
  }
}

void ConfigureContactPipeline32BatchOptions(
    SolverBenchmarkOptions& storage,
    const dart::test::LcpSolverManifestEntry& solver)
{
  if (solver.name == "Pgs" || solver.name == "Jacobi"
      || solver.name == "BlockedJacobi" || solver.name == "BGS"
      || solver.name == "RedBlackGaussSeidel"
      || solver.name == "ShockPropagation" || solver.name == "Tgs") {
    storage.options.maxIterations = 2048;
  } else if (solver.name == "SymmetricPsor") {
    storage.options.maxIterations = 512;
  } else if (solver.name == "NNCG") {
    storage.options.maxIterations = 4096;
    storage.nncgParams.pgsIterations = 160;
  }
}

void AddContactPipeline32BatchSolverCounters(
    benchmark::State& state,
    const SolverBenchmarkOptions& storage,
    const dart::test::LcpSolverManifestEntry& solver)
{
  if (solver.name == "Pgs") {
    AddPgsCounters(state, storage.options);
  } else if (solver.name == "SymmetricPsor") {
    AddSymmetricPsorCounters(state, storage.options);
  } else if (solver.name == "Jacobi") {
    AddJacobiCounters(state, storage.options);
  } else if (solver.name == "RedBlackGaussSeidel") {
    AddRedBlackGaussSeidelCounters(state, storage.options);
  } else if (solver.name == "BlockedJacobi") {
    AddBlockedJacobiCounters(state, storage.options);
  } else if (solver.name == "BGS") {
    AddBgsCounters(state, storage.options);
  } else if (solver.name == "NNCG") {
    AddNncgCounters(state, storage.nncgParams);
  } else if (solver.name == "Tgs") {
    AddTgsCounters(state, storage.options);
  }
}

template <typename Params>
void ConfigureNewtonWarmStartParameters(
    Params& params, const NewtonWarmStartMode mode)
{
  if (usesPgsWarmStart(mode)) {
    params.maxPgsWarmStartIterations = kNewtonWarmStartPgsIterations;
  }

  if (usesGradientWarmStart(mode)) {
    params.maxGradientDescentWarmStartSteps
        = kNewtonWarmStartGradientIterations;
  }
}

void ConfigureNewtonWarmStartBenchmarkOptions(
    NewtonWarmStartBenchmarkOptions& storage,
    const dart::test::LcpSolverManifestEntry& solver,
    const NewtonWarmStartMode mode)
{
  storage.options = MakeBenchmarkOptions(getBenchmarkMaxIterations(solver));

  if (solver.name == "MinimumMapNewton") {
    ConfigureNewtonWarmStartParameters(storage.minimumMapParams, mode);
    storage.options.customOptions = &storage.minimumMapParams;
  } else if (solver.name == "FischerBurmeisterNewton") {
    ConfigureNewtonWarmStartParameters(storage.fischerBurmeisterParams, mode);
    storage.options.customOptions = &storage.fischerBurmeisterParams;
  } else if (solver.name == "PenalizedFischerBurmeisterNewton") {
    storage.penalizedFischerBurmeisterParams.lambda = 1.0;
    ConfigureNewtonWarmStartParameters(
        storage.penalizedFischerBurmeisterParams, mode);
    storage.options.customOptions = &storage.penalizedFischerBurmeisterParams;
  }
}

void AddNewtonWarmStartBenchmarkCounters(
    benchmark::State& state, const NewtonWarmStartMode mode)
{
  state.counters["active_set_transition"] = 1.0;
  state.counters["newton_warm_start_mode"]
      = static_cast<double>(static_cast<int>(mode));
  state.counters["newton_pgs_warm_start"] = usesPgsWarmStart(mode) ? 1.0 : 0.0;
  state.counters["newton_gradient_warm_start"]
      = usesGradientWarmStart(mode) ? 1.0 : 0.0;
  state.counters["newton_pgs_warm_start_iterations"]
      = usesPgsWarmStart(mode) ? kNewtonWarmStartPgsIterations : 0.0;
  state.counters["newton_gradient_warm_start_iterations"]
      = usesGradientWarmStart(mode) ? kNewtonWarmStartGradientIterations : 0.0;
}

std::vector<LcpProblem> MakeBenchmarkProblemBatch(
    BenchmarkProblemFamily family, int problemArg, int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  const unsigned familySeedBase
      = 70'001u + 10'000u * static_cast<unsigned>(family);
  for (const int i : std::views::iota(0, batchSize)) {
    const unsigned seed = familySeedBase + static_cast<unsigned>(997 * i);
    switch (family) {
      case BenchmarkProblemFamily::Standard:
        problems.push_back(MakeStandardSpdProblem(problemArg, seed));
        break;
      case BenchmarkProblemFamily::Boxed:
        problems.push_back(MakeBoxedActiveBoundsProblem(problemArg, seed));
        break;
      case BenchmarkProblemFamily::FrictionIndex:
        problems.push_back(MakeFrictionIndexProblem(problemArg, seed));
        break;
    }
  }

  return problems;
}

struct GroupedBenchmarkBatch
{
  std::vector<std::vector<LcpProblem>> problemGroups;
  int problemCount{0};
  std::size_t totalProblemSize{0};
  std::size_t minProblemSize{0};
  std::size_t maxProblemSize{0};
  int totalContactCount{0};
  int minContactCount{0};
  int maxContactCount{0};
};

std::optional<GroupedBenchmarkBatch> MakeGroupedBenchmarkBatch(
    BenchmarkProblemFamily family,
    int variantsPerProblemArg,
    std::string& errorMessage)
{
  if (variantsPerProblemArg <= 0) {
    errorMessage = "variants per synthetic group must be positive";
    return std::nullopt;
  }

  GroupedBenchmarkBatch grouped;
  const auto appendGroup = [&](const int problemArg) {
    auto problems
        = MakeBenchmarkProblemBatch(family, problemArg, variantsPerProblemArg);
    const auto problemSize
        = static_cast<std::size_t>(problems.front().b.size());
    grouped.problemCount += variantsPerProblemArg;
    grouped.totalProblemSize
        += problemSize * static_cast<std::size_t>(variantsPerProblemArg);
    if (grouped.minProblemSize == 0 || problemSize < grouped.minProblemSize) {
      grouped.minProblemSize = problemSize;
    }
    grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);

    if (family == BenchmarkProblemFamily::FrictionIndex) {
      grouped.totalContactCount += problemArg * variantsPerProblemArg;
      if (grouped.minContactCount == 0
          || problemArg < grouped.minContactCount) {
        grouped.minContactCount = problemArg;
      }
      grouped.maxContactCount = std::max(grouped.maxContactCount, problemArg);
    }

    grouped.problemGroups.push_back(std::move(problems));
  };

  switch (family) {
    case BenchmarkProblemFamily::Standard:
    case BenchmarkProblemFamily::Boxed:
      for (const int rows : std::array<int, 7>{16, 32, 48, 96, 128, 192, 256}) {
        appendGroup(rows);
      }
      break;
    case BenchmarkProblemFamily::FrictionIndex:
      for (const int contactCount :
           std::array<int, 7>{4, 8, 16, 32, 48, 64, 96}) {
        appendGroup(contactCount);
      }
      break;
  }

  return grouped;
}

std::vector<LcpProblem> FlattenGroupedBenchmarkBatch(
    const GroupedBenchmarkBatch& batch)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batch.problemCount));
  for (const auto& group : batch.problemGroups) {
    problems.insert(problems.end(), group.begin(), group.end());
  }
  return problems;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
cuda_compute::LcpBatchCudaProblem MakeCudaBatchProblem(
    const std::vector<LcpProblem>& problems, std::size_t iterations)
{
  cuda_compute::LcpBatchCudaProblem packet;
  if (problems.empty()) {
    packet.iterations = iterations;
    return packet;
  }

  const auto rows = static_cast<std::size_t>(problems.front().b.size());
  packet.problemSize = rows;
  packet.problemCount = problems.size();
  packet.iterations = iterations;
  packet.A.reserve(rows * rows * problems.size());
  packet.b.reserve(rows * problems.size());
  packet.lo.reserve(rows * problems.size());
  packet.hi.reserve(rows * problems.size());
  packet.findex.reserve(rows * problems.size());
  packet.x.assign(rows * problems.size(), 0.0);

  for (const auto& problem : problems) {
    for (const auto r : std::views::iota(std::size_t{0}, rows)) {
      for (const auto c : std::views::iota(std::size_t{0}, rows)) {
        packet.A.push_back(problem.A(
            static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)));
      }
    }

    for (const auto i : std::views::iota(std::size_t{0}, rows)) {
      packet.b.push_back(problem.b[static_cast<Eigen::Index>(i)]);
      packet.lo.push_back(problem.lo[static_cast<Eigen::Index>(i)]);
      packet.hi.push_back(problem.hi[static_cast<Eigen::Index>(i)]);
      packet.findex.push_back(problem.findex[static_cast<Eigen::Index>(i)]);
    }
  }

  return packet;
}

struct CudaGroupedBenchmarkBatch
{
  GroupedBenchmarkBatch grouped;
  std::vector<cuda_compute::LcpBatchCudaProblem> packets;
};

std::optional<CudaGroupedBenchmarkBatch> MakeGroupedCudaBenchmarkBatch(
    BenchmarkProblemFamily family,
    int variantsPerProblemArg,
    std::size_t iterations,
    std::string& errorMessage)
{
  auto grouped
      = MakeGroupedBenchmarkBatch(family, variantsPerProblemArg, errorMessage);
  if (!grouped.has_value()) {
    return std::nullopt;
  }

  CudaGroupedBenchmarkBatch cudaBatch;
  cudaBatch.grouped = std::move(*grouped);
  cudaBatch.packets.reserve(cudaBatch.grouped.problemGroups.size());
  for (const auto& problemGroup : cudaBatch.grouped.problemGroups) {
    cudaBatch.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return cudaBatch;
}

struct CudaGroupedWorldContactBenchmarkBatch
{
  WorldContactBenchmarkBatch aggregate;
  std::vector<std::vector<LcpProblem>> problemGroups;
  std::vector<cuda_compute::LcpBatchCudaProblem> packets;
  std::size_t minProblemSize{0};
  std::size_t maxProblemSize{0};
};

std::optional<CudaGroupedWorldContactBenchmarkBatch>
MakeGroupedWorldContactCudaBatch(
    int variantsPerContactCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerContactCount <= 0) {
    errorMessage = "variants per contact count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 7> kContactCounts{1, 2, 4, 8, 16, 24, 32};

  CudaGroupedWorldContactBenchmarkBatch grouped;
  std::vector<std::size_t> groupSizes;
  grouped.aggregate.problems.reserve(
      kContactCounts.size()
      * static_cast<std::size_t>(variantsPerContactCount));

  int variantBase = 0;
  for (const int contactCount : kContactCounts) {
    for (const int variant : std::views::iota(0, variantsPerContactCount)) {
      auto fixture = MakeWorldContactBenchmarkProblem(
          contactCount, errorMessage, variantBase + variant);
      if (!fixture.has_value()) {
        return std::nullopt;
      }

      const auto problemSize
          = static_cast<std::size_t>(fixture->problem.b.size());
      if (problemSize != static_cast<std::size_t>(3 * contactCount)) {
        errorMessage = "grouped CUDA world-contact problem shape changed";
        return std::nullopt;
      }

      grouped.aggregate.totalProblemSize += fixture->problem.b.size();
      grouped.aggregate.totalContactCount += fixture->contactCount;
      grouped.aggregate.totalBodyCount += fixture->bodyCount;
      grouped.aggregate.problems.push_back(fixture->problem);

      auto groupIt = std::ranges::find(groupSizes, problemSize);
      if (groupIt == groupSizes.end()) {
        groupSizes.push_back(problemSize);
        grouped.problemGroups.emplace_back();
        groupIt = std::prev(groupSizes.end());
      }

      const auto groupIndex = static_cast<std::size_t>(
          std::distance(groupSizes.begin(), groupIt));
      grouped.problemGroups[groupIndex].push_back(std::move(fixture->problem));
      if (grouped.minProblemSize == 0 || problemSize < grouped.minProblemSize) {
        grouped.minProblemSize = problemSize;
      }
      grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);
    }
    variantBase += variantsPerContactCount;
  }

  grouped.packets.reserve(grouped.problemGroups.size());
  for (const auto& problemGroup : grouped.problemGroups) {
    grouped.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return grouped;
}

std::optional<CudaGroupedWorldContactBenchmarkBatch>
MakeGroupedWorldBoxContactCudaBatch(
    int variantsPerBoxCount, std::size_t iterations, std::string& errorMessage)
{
  if (variantsPerBoxCount <= 0) {
    errorMessage = "variants per dense box count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 10> kBoxCounts{1, 2, 4, 8, 16, 24, 32, 48, 64, 96};

  CudaGroupedWorldContactBenchmarkBatch grouped;
  std::vector<std::size_t> groupSizes;
  grouped.aggregate.problems.reserve(
      kBoxCounts.size() * static_cast<std::size_t>(variantsPerBoxCount));

  int variantBase = 0;
  for (const int boxCount : kBoxCounts) {
    for (const int variant : std::views::iota(0, variantsPerBoxCount)) {
      auto fixture = MakeWorldBoxContactBenchmarkProblem(
          errorMessage, variantBase + variant, boxCount);
      if (!fixture.has_value()) {
        return std::nullopt;
      }

      const auto problemSize
          = static_cast<std::size_t>(fixture->problem.b.size());
      if (fixture->contactCount != static_cast<std::size_t>(4 * boxCount)
          || problemSize != static_cast<std::size_t>(12 * boxCount)) {
        errorMessage = "grouped CUDA dense box-contact shape changed";
        return std::nullopt;
      }

      grouped.aggregate.totalProblemSize += fixture->problem.b.size();
      grouped.aggregate.totalContactCount += fixture->contactCount;
      grouped.aggregate.totalBodyCount += fixture->bodyCount;
      grouped.aggregate.problems.push_back(fixture->problem);

      auto groupIt = std::ranges::find(groupSizes, problemSize);
      if (groupIt == groupSizes.end()) {
        groupSizes.push_back(problemSize);
        grouped.problemGroups.emplace_back();
        groupIt = std::prev(groupSizes.end());
      }

      const auto groupIndex = static_cast<std::size_t>(
          std::distance(groupSizes.begin(), groupIt));
      grouped.problemGroups[groupIndex].push_back(std::move(fixture->problem));
      if (grouped.minProblemSize == 0 || problemSize < grouped.minProblemSize) {
        grouped.minProblemSize = problemSize;
      }
      grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);
    }
    variantBase += variantsPerBoxCount;
  }

  grouped.packets.reserve(grouped.problemGroups.size());
  for (const auto& problemGroup : grouped.problemGroups) {
    grouped.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return grouped;
}

std::optional<CudaGroupedWorldContactBenchmarkBatch>
MakeGroupedWorldStackContactCudaBatch(
    int variantsPerSphereCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerSphereCount <= 0) {
    errorMessage = "variants per sphere count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 17> kSphereCounts{
      2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 32};

  CudaGroupedWorldContactBenchmarkBatch grouped;
  std::vector<std::size_t> groupSizes;
  grouped.aggregate.problems.reserve(
      kSphereCounts.size() * static_cast<std::size_t>(variantsPerSphereCount));

  int variantBase = 0;
  for (const int sphereCount : kSphereCounts) {
    for (const int variant : std::views::iota(0, variantsPerSphereCount)) {
      auto fixture = MakeWorldStackContactBenchmarkProblem(
          sphereCount, errorMessage, variantBase + variant);
      if (!fixture.has_value()) {
        return std::nullopt;
      }

      const auto problemSize
          = static_cast<std::size_t>(fixture->problem.b.size());
      if (problemSize != static_cast<std::size_t>(3 * sphereCount)) {
        errorMessage = "grouped CUDA world-stack problem shape changed";
        return std::nullopt;
      }

      grouped.aggregate.totalProblemSize += fixture->problem.b.size();
      grouped.aggregate.totalContactCount += fixture->contactCount;
      grouped.aggregate.totalBodyCount += fixture->bodyCount;
      grouped.aggregate.problems.push_back(fixture->problem);

      auto groupIt = std::ranges::find(groupSizes, problemSize);
      if (groupIt == groupSizes.end()) {
        groupSizes.push_back(problemSize);
        grouped.problemGroups.emplace_back();
        groupIt = std::prev(groupSizes.end());
      }

      const auto groupIndex = static_cast<std::size_t>(
          std::distance(groupSizes.begin(), groupIt));
      grouped.problemGroups[groupIndex].push_back(std::move(fixture->problem));
      if (grouped.minProblemSize == 0 || problemSize < grouped.minProblemSize) {
        grouped.minProblemSize = problemSize;
      }
      grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);
    }
    variantBase += variantsPerSphereCount;
  }

  grouped.packets.reserve(grouped.problemGroups.size());
  for (const auto& problemGroup : grouped.problemGroups) {
    grouped.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return grouped;
}

std::optional<CudaGroupedWorldContactBenchmarkBatch>
MakeGroupedArticulatedUnifiedContactCudaBatch(
    int variantsPerContactCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerContactCount <= 0) {
    errorMessage = "variants per articulated contact count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 6> kContactCounts{1, 4, 8, 16, 24, 32};
  CudaGroupedWorldContactBenchmarkBatch grouped;
  std::vector<std::size_t> groupSizes;
  grouped.aggregate.problems.reserve(
      3 * kContactCounts.size()
      * static_cast<std::size_t>(variantsPerContactCount));

  int variantBase = 0;
  for (const int contactCount : kContactCounts) {
    for (const int variant : std::views::iota(0, variantsPerContactCount)) {
      for (const ArticulatedContactBenchmarkCase benchmarkCase :
           {ArticulatedContactBenchmarkCase::Ground,
            ArticulatedContactBenchmarkCase::RigidImpact,
            ArticulatedContactBenchmarkCase::CrossLinkImpact}) {
        auto fixture = MakeArticulatedUnifiedContactBenchmarkProblem(
            benchmarkCase, contactCount, errorMessage, variantBase + variant);
        if (!fixture.has_value()) {
          return std::nullopt;
        }

        const auto problemSize
            = static_cast<std::size_t>(fixture->problem.b.size());
        if (problemSize != static_cast<std::size_t>(3 * contactCount)) {
          errorMessage = "grouped CUDA articulated contact shape changed";
          return std::nullopt;
        }

        grouped.aggregate.totalProblemSize += fixture->problem.b.size();
        grouped.aggregate.totalContactCount += fixture->contactCount;
        grouped.aggregate.totalBodyCount += fixture->bodyCount;
        grouped.aggregate.problems.push_back(fixture->problem);

        auto groupIt = std::ranges::find(groupSizes, problemSize);
        if (groupIt == groupSizes.end()) {
          groupSizes.push_back(problemSize);
          grouped.problemGroups.emplace_back();
          groupIt = std::prev(groupSizes.end());
        }

        const auto groupIndex = static_cast<std::size_t>(
            std::distance(groupSizes.begin(), groupIt));
        grouped.problemGroups[groupIndex].push_back(
            std::move(fixture->problem));
        if (grouped.minProblemSize == 0
            || problemSize < grouped.minProblemSize) {
          grouped.minProblemSize = problemSize;
        }
        grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);
      }
    }
    variantBase += variantsPerContactCount;
  }

  grouped.packets.reserve(grouped.problemGroups.size());
  for (const auto& problemGroup : grouped.problemGroups) {
    grouped.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return grouped;
}

std::optional<CudaGroupedWorldContactBenchmarkBatch>
MakeGroupedMixedContactCudaBatch(
    int variantsPerScenario, std::size_t iterations, std::string& errorMessage)
{
  if (variantsPerScenario <= 0) {
    errorMessage = "variants per mixed contact scenario must be positive";
    return std::nullopt;
  }

  CudaGroupedWorldContactBenchmarkBatch grouped;
  std::vector<std::size_t> groupSizes;
  grouped.aggregate.problems.reserve(
      22 * static_cast<std::size_t>(variantsPerScenario));

  auto appendFixture =
      [&](std::optional<WorldContactBenchmarkProblem> fixture) {
        if (!fixture.has_value()) {
          return false;
        }

        const auto problemSize
            = static_cast<std::size_t>(fixture->problem.b.size());
        grouped.aggregate.totalProblemSize += fixture->problem.b.size();
        grouped.aggregate.totalContactCount += fixture->contactCount;
        grouped.aggregate.totalBodyCount += fixture->bodyCount;
        grouped.aggregate.problems.push_back(fixture->problem);

        auto groupIt = std::ranges::find(groupSizes, problemSize);
        if (groupIt == groupSizes.end()) {
          groupSizes.push_back(problemSize);
          grouped.problemGroups.emplace_back();
          groupIt = std::prev(groupSizes.end());
        }

        const auto groupIndex = static_cast<std::size_t>(
            std::distance(groupSizes.begin(), groupIt));
        grouped.problemGroups[groupIndex].push_back(
            std::move(fixture->problem));
        if (grouped.minProblemSize == 0
            || problemSize < grouped.minProblemSize) {
          grouped.minProblemSize = problemSize;
        }
        grouped.maxProblemSize = std::max(grouped.maxProblemSize, problemSize);
        return true;
      };

  for (const int variant : std::views::iota(0, variantsPerScenario)) {
    if (!appendFixture(
            MakeWorldContactBenchmarkProblem(1, errorMessage, variant))
        || !appendFixture(
            MakeWorldContactBenchmarkProblem(4, errorMessage, variant))
        || !appendFixture(
            MakeWorldStackContactBenchmarkProblem(2, errorMessage, variant))
        || !appendFixture(
            MakeWorldStackContactBenchmarkProblem(5, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 1, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            1,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            1,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 4, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            4,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            4,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 8, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            8,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            8,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 16, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            16,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            16,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 24, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            24,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            24,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::Ground, 32, errorMessage, variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::RigidImpact,
            32,
            errorMessage,
            variant))
        || !appendFixture(MakeArticulatedUnifiedContactBenchmarkProblem(
            ArticulatedContactBenchmarkCase::CrossLinkImpact,
            32,
            errorMessage,
            variant))) {
      return std::nullopt;
    }
  }

  grouped.packets.reserve(grouped.problemGroups.size());
  for (const auto& problemGroup : grouped.problemGroups) {
    grouped.packets.push_back(MakeCudaBatchProblem(problemGroup, iterations));
  }

  return grouped;
}

BatchBenchmarkCounters CheckCudaBatchResult(
    const std::vector<LcpProblem>& problems,
    const std::vector<double>& solutions,
    const LcpOptions& options,
    std::size_t iterations)
{
  BatchBenchmarkCounters counters;
  if (problems.empty()) {
    return counters;
  }

  const auto rows = static_cast<std::size_t>(problems.front().b.size());
  for (const auto problemIndex :
       std::views::iota(std::size_t{0}, problems.size())) {
    Eigen::VectorXd x(static_cast<Eigen::Index>(rows));
    for (const auto row : std::views::iota(std::size_t{0}, rows)) {
      x[static_cast<Eigen::Index>(row)] = solutions[problemIndex * rows + row];
    }

    const auto check
        = dart::test::CheckLcpSolution(problems[problemIndex], x, options);
    ++counters.problemCount;
    counters.totalIterations += static_cast<double>(iterations);
    counters.maxResidual = std::max(counters.maxResidual, check.residual);
    counters.maxComplementarity
        = std::max(counters.maxComplementarity, check.complementarity);
    counters.maxBoundViolation
        = std::max(counters.maxBoundViolation, check.boundViolation);
    counters.allContractsOk = counters.allContractsOk && check.ok;
  }

  return counters;
}

void MergeBatchBenchmarkCounters(
    BatchBenchmarkCounters& total, const BatchBenchmarkCounters& group)
{
  total.problemCount += group.problemCount;
  total.totalIterations += group.totalIterations;
  total.maxResidual = std::max(total.maxResidual, group.maxResidual);
  total.maxComplementarity
      = std::max(total.maxComplementarity, group.maxComplementarity);
  total.maxBoundViolation
      = std::max(total.maxBoundViolation, group.maxBoundViolation);
  total.allContractsOk = total.allContractsOk && group.allContractsOk;
}

BatchBenchmarkCounters CheckCudaGroupedBatchResult(
    const std::vector<std::vector<LcpProblem>>& problemGroups,
    const std::vector<cuda_compute::LcpBatchCudaProblem>& packets,
    const LcpOptions& options,
    std::size_t iterations)
{
  BatchBenchmarkCounters counters;
  if (problemGroups.size() != packets.size()) {
    counters.allContractsOk = false;
    return counters;
  }

  for (const auto groupIndex :
       std::views::iota(std::size_t{0}, problemGroups.size())) {
    const auto groupCounters = CheckCudaBatchResult(
        problemGroups[groupIndex], packets[groupIndex].x, options, iterations);
    MergeBatchBenchmarkCounters(counters, groupCounters);
  }

  return counters;
}
#endif

void AccumulateBatchResult(
    BatchBenchmarkCounters& counters,
    const dart::math::LcpResult& result,
    const dart::test::LcpCheckResult& check)
{
  ++counters.problemCount;
  counters.totalIterations += static_cast<double>(result.iterations);
  counters.maxResidual = std::max(counters.maxResidual, check.residual);
  counters.maxComplementarity
      = std::max(counters.maxComplementarity, check.complementarity);
  counters.maxBoundViolation
      = std::max(counters.maxBoundViolation, check.boundViolation);
  counters.allContractsOk = counters.allContractsOk && check.ok;
}

BatchBenchmarkCounters RunBatchWithSolver(
    dart::math::LcpSolver& solver,
    const std::vector<LcpProblem>& problems,
    const LcpOptions& options)
{
  BatchBenchmarkCounters counters;

  for (const auto& problem : problems) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(problem.b.size());
    const auto result = solver.solve(problem, x, options);
    const auto check = dart::test::CheckLcpSolution(problem, x, options);
    AccumulateBatchResult(counters, result, check);
    auto status = result.status;
    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(x.data());
  }

  return counters;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
struct ParallelBatchFixture
{
  ParallelBatchFixture(
      const dart::test::LcpSolverManifestEntry& solverEntry,
      BenchmarkProblemFamily family,
      int problemArg,
      int batchSize)
    : ParallelBatchFixture(
          solverEntry, MakeBenchmarkProblemBatch(family, problemArg, batchSize))
  {
    // Delegate to the explicit-problem constructor so synthetic and
    // contact-derived batch rows use identical compute-graph execution.
  }

  ParallelBatchFixture(
      const dart::test::LcpSolverManifestEntry& solverEntry,
      std::vector<LcpProblem> inputProblems)
    : problems(std::move(inputProblems)),
      solutions(problems.size()),
      results(problems.size()),
      checks(problems.size())
  {
    if (problems.empty()) {
      valid = false;
      errorMessage = "LCP batch fixture has no problems";
      return;
    }

    ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());

    solvers.reserve(problems.size());
    for (const std::size_t index :
         std::views::iota(std::size_t{0}, problems.size())) {
      auto solver = solverEntry.create();
      if (solver == nullptr) {
        valid = false;
        errorMessage = "LCP solver factory returned null";
        return;
      }

      solvers.push_back(std::move(solver));
      solutions[index] = Eigen::VectorXd::Zero(problems[index].b.size());

      compute::ComputeStageMetadata metadata;
      metadata.domain = compute::ComputeStageDomain::Constraint;
      metadata.acceleration = compute::ComputeStageAcceleration::TaskParallel
                              | compute::ComputeStageAcceleration::DataParallel;
      metadata.resources.push_back(
          {"lcp_batch_problem_" + std::to_string(index),
           compute::ComputeAccessMode::ReadWrite});

      graph.addNode(
          "problem_" + std::to_string(index),
          [this, index]() { solveProblem(index); },
          metadata);
    }
  }

  void solveProblem(std::size_t index)
  {
    solutions[index].setZero();
    results[index] = solvers[index]->solve(
        problems[index], solutions[index], storage.options);
    checks[index] = dart::test::CheckLcpSolution(
        problems[index], solutions[index], storage.options);
  }

  BatchBenchmarkCounters collectCounters() const
  {
    BatchBenchmarkCounters counters;
    for (std::size_t i = 0; i < results.size(); ++i) {
      AccumulateBatchResult(counters, results[i], checks[i]);
    }
    return counters;
  }

  std::vector<LcpProblem> problems;
  SolverBenchmarkOptions storage;
  std::vector<std::unique_ptr<dart::math::LcpSolver>> solvers;
  std::vector<Eigen::VectorXd> solutions;
  std::vector<dart::math::LcpResult> results;
  std::vector<dart::test::LcpCheckResult> checks;
  compute::ComputeGraph graph;
  bool valid{true};
  std::string errorMessage;
};

struct ParallelNewtonWarmStartBatchFixture
{
  ParallelNewtonWarmStartBatchFixture(
      const dart::test::LcpSolverManifestEntry& solverEntry,
      const NewtonWarmStartMode mode,
      std::vector<LcpProblem> inputProblems)
    : problems(std::move(inputProblems)),
      solutions(problems.size()),
      results(problems.size()),
      checks(problems.size())
  {
    if (problems.empty()) {
      valid = false;
      errorMessage = "LCP batch fixture has no problems";
      return;
    }

    ConfigureNewtonWarmStartBenchmarkOptions(storage, solverEntry, mode);

    solvers.reserve(problems.size());
    for (const std::size_t index :
         std::views::iota(std::size_t{0}, problems.size())) {
      auto solver = solverEntry.create();
      if (solver == nullptr) {
        valid = false;
        errorMessage = "LCP solver factory returned null";
        return;
      }
      if (!solver->supportsProblem(problems[index])) {
        valid = false;
        errorMessage
            = "Newton warm-start parallel batch case exceeds concrete solver "
              "support";
        return;
      }

      solvers.push_back(std::move(solver));
      solutions[index] = Eigen::VectorXd::Zero(problems[index].b.size());

      compute::ComputeStageMetadata metadata;
      metadata.domain = compute::ComputeStageDomain::Constraint;
      metadata.acceleration = compute::ComputeStageAcceleration::TaskParallel
                              | compute::ComputeStageAcceleration::DataParallel;
      metadata.resources.push_back(
          {"newton_warm_start_lcp_batch_problem_" + std::to_string(index),
           compute::ComputeAccessMode::ReadWrite});

      graph.addNode(
          "newton_warm_start_problem_" + std::to_string(index),
          [this, index]() { solveProblem(index); },
          metadata);
    }
  }

  void solveProblem(std::size_t index)
  {
    solutions[index].setZero();
    results[index] = solvers[index]->solve(
        problems[index], solutions[index], storage.options);
    checks[index] = dart::test::CheckLcpSolution(
        problems[index], solutions[index], storage.options);
  }

  BatchBenchmarkCounters collectCounters() const
  {
    BatchBenchmarkCounters counters;
    for (std::size_t i = 0; i < results.size(); ++i) {
      AccumulateBatchResult(counters, results[i], checks[i]);
    }
    return counters;
  }

  std::vector<LcpProblem> problems;
  NewtonWarmStartBenchmarkOptions storage;
  std::vector<std::unique_ptr<dart::math::LcpSolver>> solvers;
  std::vector<Eigen::VectorXd> solutions;
  std::vector<dart::math::LcpResult> results;
  std::vector<dart::test::LcpCheckResult> checks;
  compute::ComputeGraph graph;
  bool valid{true};
  std::string errorMessage;
};
#endif

void AddBatchBenchmarkCounters(
    benchmark::State& state,
    const BatchBenchmarkCounters& counters,
    int rowsPerProblem,
    int batchSize,
    const std::string& label)
{
  state.counters["batch_size"] = batchSize;
  state.counters["problem_size"] = rowsPerProblem;
  state.counters["total_problem_size"] = rowsPerProblem * batchSize;
  state.counters["avg_iterations"]
      = counters.problemCount > 0
            ? counters.totalIterations
                  / static_cast<double>(counters.problemCount)
            : 0.0;
  state.counters["max_residual"] = counters.maxResidual;
  state.counters["max_complementarity"] = counters.maxComplementarity;
  state.counters["max_bound_violation"] = counters.maxBoundViolation;
  state.counters["contract_ok"] = counters.allContractsOk ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * rowsPerProblem * batchSize);
  state.SetLabel(label);
}

void AddGroupedBatchBenchmarkCounters(
    benchmark::State& state,
    const BatchBenchmarkCounters& counters,
    const GroupedBenchmarkBatch& batch,
    const std::string& label)
{
  state.counters["batch_size"] = batch.problemCount;
  state.counters["batch_group_count"]
      = static_cast<double>(batch.problemGroups.size());
  state.counters["min_problem_size"]
      = static_cast<double>(batch.minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch.maxProblemSize);
  state.counters["total_problem_size"]
      = static_cast<double>(batch.totalProblemSize);
  state.counters["avg_iterations"]
      = counters.problemCount > 0
            ? counters.totalIterations
                  / static_cast<double>(counters.problemCount)
            : 0.0;
  state.counters["max_residual"] = counters.maxResidual;
  state.counters["max_complementarity"] = counters.maxComplementarity;
  state.counters["max_bound_violation"] = counters.maxBoundViolation;
  state.counters["contract_ok"] = counters.allContractsOk ? 1.0 : 0.0;
  if (batch.totalContactCount > 0) {
    state.counters["min_contact_count"] = batch.minContactCount;
    state.counters["max_contact_count"] = batch.maxContactCount;
    state.counters["total_contact_count"] = batch.totalContactCount;
  }
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * batch.totalProblemSize);
  state.SetLabel(label);
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
void AddCudaGroupedBenchmarkCounters(
    benchmark::State& state,
    const BatchBenchmarkCounters& counters,
    const CudaGroupedBenchmarkBatch& batch,
    const std::string& label)
{
  AddGroupedBatchBenchmarkCounters(state, counters, batch.grouped, label);
  state.counters["cuda_group_count"]
      = static_cast<double>(batch.grouped.problemGroups.size());
}
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void AddWorldContactBatchCounters(
    benchmark::State& state,
    const BatchBenchmarkCounters& counters,
    const WorldContactBenchmarkBatch& batch,
    const std::string& label)
{
  state.counters["batch_size"] = static_cast<double>(batch.problems.size());
  state.counters["total_problem_size"]
      = static_cast<double>(batch.totalProblemSize);
  state.counters["total_contact_count"]
      = static_cast<double>(batch.totalContactCount);
  state.counters["total_body_count"]
      = static_cast<double>(batch.totalBodyCount);
  state.counters["avg_iterations"]
      = counters.problemCount > 0
            ? counters.totalIterations
                  / static_cast<double>(counters.problemCount)
            : 0.0;
  state.counters["max_residual"] = counters.maxResidual;
  state.counters["max_complementarity"] = counters.maxComplementarity;
  state.counters["max_bound_violation"] = counters.maxBoundViolation;
  state.counters["contract_ok"] = counters.allContractsOk ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * batch.totalProblemSize);
  state.SetLabel(label);
}

void AddParallelExecutionCounters(
    benchmark::State& state,
    const compute::ComputeExecutionProfile& profile,
    const compute::ComputeGraph& graph)
{
  const bool profiled = !profile.isEmpty();
  state.counters["parallel_units"] = graph.getNodeCount();
  state.counters["profile_enabled"] = profiled ? 1.0 : 0.0;
  state.counters["worker_count"]
      = profiled ? static_cast<double>(profile.workerCount) : 0.0;
  state.counters["max_parallelism"]
      = profiled ? static_cast<double>(profile.maxParallelism) : 0.0;
  state.counters["avg_parallelism"]
      = profiled ? profile.getAverageParallelism() : 0.0;
}
#endif

void RunManifestBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const int problemArg = static_cast<int>(state.range(0));
  const auto problem = MakeBenchmarkProblem(family, problemArg);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  if (solverEntry.name == "BoxedSemiSmoothNewton"
      && family == BenchmarkProblemFamily::FrictionIndex
      && storage.hasBoxedSsnParams) {
    storage.boxedSsnParams.maxFrictionIndexExactSolveDimension = 192;
  }

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(problem)) {
    state.SkipWithError(
        "Manifest benchmark case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          std::string(getProblemFamilyName(family))));
  AddSolverIdentityCounters(state, solverEntry);

  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
  if (storage.hasSapParams) {
    AddSapCounters(state, storage.sapParams);
  }
  if (storage.hasBoxedSsnParams) {
    AddBoxedSsnCounters(state, storage.boxedSsnParams);
  }
}

void RunActiveSetTransitionBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const auto problem = MakeActiveSetTransitionBenchmarkProblem(family);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(problem)) {
    state.SkipWithError(
        "Active-set transition case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "ActiveSetTransition/" + std::string(getProblemFamilyName(family))));

  state.counters["active_set_transition"] = 1.0;
  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = 6.0;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

void RunActiveFrictionIndexContactBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const auto problem = MakeActiveFrictionIndexContactProblem();
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(problem)) {
    state.SkipWithError(
        "Active friction-index contact case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "ActiveFrictionIndexContact/FrictionIndex"));

  state.counters["active_friction_index_contact"] = 1.0;
  state.counters["contact_count"] = 2.0;
  state.counters["normal_row_count"] = 2.0;
  state.counters["tangent_row_count"] = 4.0;
  state.counters["active_tangent_bound_count"] = 2.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

void RunPgsRelaxationSweepBenchmark(
    benchmark::State& state, const RelaxationSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(200);
  options.relaxation = testCase.relaxation;

  dart::math::PgsSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "Pgs",
          "RelaxationSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.relaxationLabel)));

  state.counters["pgs_relaxation_sweep"] = 1.0;
  state.counters["psor_relaxation"] = testCase.relaxation;
  state.counters["psor_under_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Under ? 1.0 : 0.0;
  state.counters["psor_plain_pgs"]
      = testCase.relaxationKind == RelaxationSweepKind::Plain ? 1.0 : 0.0;
  state.counters["psor_over_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Over ? 1.0 : 0.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunSymmetricPsorRelaxationSweepBenchmark(
    benchmark::State& state, const RelaxationSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(200);
  options.relaxation = testCase.relaxation;

  dart::math::SymmetricPsorSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "SymmetricPsor",
          "RelaxationSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.relaxationLabel)));

  state.counters["symmetric_psor_relaxation_sweep"] = 1.0;
  state.counters["psor_relaxation"] = testCase.relaxation;
  state.counters["psor_under_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Under ? 1.0 : 0.0;
  state.counters["psor_plain_symmetric_psor"]
      = testCase.relaxationKind == RelaxationSweepKind::Plain ? 1.0 : 0.0;
  state.counters["psor_over_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Over ? 1.0 : 0.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunRedBlackGaussSeidelRelaxationSweepBenchmark(
    benchmark::State& state, const RelaxationSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(200);
  options.relaxation = testCase.relaxation;

  dart::math::RedBlackGaussSeidelSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "RedBlackGaussSeidel",
          "RelaxationSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.relaxationLabel)));

  const double problemSize = static_cast<double>(problem.b.size());
  state.counters["red_black_gauss_seidel_relaxation_sweep"] = 1.0;
  state.counters["red_black_color_count"] = 2.0;
  state.counters["red_black_red_rows"] = std::ceil(problemSize / 2.0);
  state.counters["red_black_black_rows"] = std::floor(problemSize / 2.0);
  state.counters["psor_relaxation"] = testCase.relaxation;
  state.counters["psor_under_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Under ? 1.0 : 0.0;
  state.counters["psor_plain_red_black_gauss_seidel"]
      = testCase.relaxationKind == RelaxationSweepKind::Plain ? 1.0 : 0.0;
  state.counters["psor_over_relaxation"]
      = testCase.relaxationKind == RelaxationSweepKind::Over ? 1.0 : 0.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunBoxedSsnLineSearchSweepBenchmark(
    benchmark::State& state, const BoxedSsnLineSearchSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(100);
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  dart::math::BoxedSemiSmoothNewtonSolver::Parameters params;
  params.maxLineSearchSteps = testCase.maxLineSearchSteps;
  params.stepReduction = testCase.stepReduction;
  options.customOptions = &params;

  dart::math::BoxedSemiSmoothNewtonSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "BoxedSemiSmoothNewton",
          "LineSearchSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.lineSearchLabel)));

  state.counters["boxed_ssn_line_search_sweep"] = 1.0;
  state.counters["boxed_ssn_max_line_search_steps"] = params.maxLineSearchSteps;
  state.counters["boxed_ssn_step_reduction"] = params.stepReduction;
  state.counters["boxed_ssn_sufficient_decrease"] = params.sufficientDecrease;
  state.counters["boxed_ssn_min_step"] = params.minStep;
  state.counters["boxed_ssn_jacobian_regularization"]
      = params.jacobianRegularization;
  state.counters["boxed_ssn_default_search"]
      = (params.maxLineSearchSteps == 10 && params.stepReduction == 0.5) ? 1.0
                                                                         : 0.0;
  state.counters["boxed_ssn_more_steps"]
      = (params.maxLineSearchSteps == 20 && params.stepReduction == 0.5) ? 1.0
                                                                         : 0.0;
  state.counters["boxed_ssn_gentle_reduction"]
      = params.stepReduction == 0.8 ? 1.0 : 0.0;
  state.counters["boxed_ssn_pgs_warm_start_iterations"]
      = params.maxPgsWarmStartIterations;
  state.counters["boxed_ssn_pgs_warm_start_relaxation"]
      = params.pgsWarmStartRelaxation;
  state.counters["boxed_ssn_friction_index_exact_solve_dimension"]
      = params.maxFrictionIndexExactSolveDimension;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunPivotingScaleSweepBenchmark(
    benchmark::State& state, const PivotingScaleSweepCase testCase)
{
  const auto* solverEntry = FindSolverManifestEntry(testCase.solverName);
  if (solverEntry == nullptr) {
    state.SkipWithError("pivoting scale sweep solver is not in the manifest");
    return;
  }

  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  if (!SolverSupportsConcreteProblem(*solverEntry, problem)) {
    state.SkipWithError(
        "pivoting scale sweep case exceeds concrete solver support");
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, *solverEntry, problem);

  const auto solver = solverEntry->create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry->name),
          "PivotingScaleSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.problemLabel)));

  state.counters["pivoting_scale_sweep"] = 1.0;
  state.counters["pivoting_problem_arg"] = testCase.problemArg;
  state.counters["pivoting_standard_family"]
      = testCase.family == BenchmarkProblemFamily::Standard ? 1.0 : 0.0;
  state.counters["pivoting_boxed_family"]
      = testCase.family == BenchmarkProblemFamily::Boxed ? 1.0 : 0.0;
  state.counters["pivoting_friction_index_family"]
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 1.0 : 0.0;
  state.counters["pivoting_direct_enumeration"]
      = solverEntry->name == "Direct" ? 1.0 : 0.0;
  state.counters["pivoting_direct_no_fallback"]
      = solverEntry->name == "Direct" && testCase.problemArg <= 3 ? 1.0 : 0.0;
  state.counters["pivoting_dantzig_boxed_or_findex"]
      = solverEntry->name == "Dantzig"
                && testCase.family != BenchmarkProblemFamily::Standard
            ? 1.0
            : 0.0;
  state.counters["pivoting_standard_only_solver"]
      = solverEntry->name != "Dantzig" ? 1.0 : 0.0;
  state.counters["pivoting_supports_boxed"]
      = solverEntry->supportsBoxed ? 1.0 : 0.0;
  state.counters["pivoting_supports_friction_index"]
      = solverEntry->supportsFrictionIndex ? 1.0 : 0.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void AddBlockPartitionSweepCounters(
    benchmark::State& state,
    const BlockPartitionSweepCase testCase,
    const LcpProblem& problem,
    const std::vector<int>& blockSizes)
{
  int blockCount = static_cast<int>(blockSizes.size());
  int minBlockSize = blockSizes.empty() ? 0 : blockSizes.front();
  int maxBlockSize = blockSizes.empty() ? 0 : blockSizes.front();
  int blockSizeSum = 0;
  for (const int blockSize : blockSizes) {
    minBlockSize = std::min(minBlockSize, blockSize);
    maxBlockSize = std::max(maxBlockSize, blockSize);
    blockSizeSum += blockSize;
  }

  if (testCase.profile == BlockPartitionProfile::AutoFindex) {
    blockCount = testCase.problemArg;
    minBlockSize = 3;
    maxBlockSize = 3;
    blockSizeSum = static_cast<int>(problem.b.size());
  }

  state.counters["block_partition_sweep"] = 1.0;
  state.counters["block_partition_problem_arg"] = testCase.problemArg;
  state.counters["block_partition_block_count"] = blockCount;
  state.counters["block_partition_min_block_size"] = minBlockSize;
  state.counters["block_partition_max_block_size"] = maxBlockSize;
  state.counters["block_partition_size_sum"] = blockSizeSum;
  state.counters["block_partition_full_block"]
      = testCase.profile == BlockPartitionProfile::FullBlock ? 1.0 : 0.0;
  state.counters["block_partition_uniform_three"]
      = testCase.profile == BlockPartitionProfile::UniformThree ? 1.0 : 0.0;
  state.counters["block_partition_auto_findex"]
      = testCase.profile == BlockPartitionProfile::AutoFindex ? 1.0 : 0.0;
  state.counters["block_partition_contact_blocks"]
      = testCase.profile == BlockPartitionProfile::ContactBlocks ? 1.0 : 0.0;
  state.counters["block_partition_bgs_solver"]
      = testCase.solverName == "BGS" ? 1.0 : 0.0;
  state.counters["block_partition_blocked_jacobi_solver"]
      = testCase.solverName == "BlockedJacobi" ? 1.0 : 0.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunBlockPartitionSweepBenchmark(
    benchmark::State& state, const BlockPartitionSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  const auto blockSizes = MakeBlockPartition(problem, testCase);
  auto options = MakeBenchmarkOptions(500);

  const std::string label = "BlockPartitionSweep/"
                            + std::string(getProblemFamilyName(testCase.family))
                            + "/" + std::string(testCase.profileLabel);

  if (testCase.solverName == "BGS") {
    dart::math::BgsSolver::Parameters params;
    params.blockSizes = blockSizes;
    options.customOptions = &params;
    dart::math::BgsSolver solver;
    RunBenchmarkWithSolver(
        state, solver, problem, options, MakeLabel("BGS", label));
  } else if (testCase.solverName == "BlockedJacobi") {
    dart::math::BlockedJacobiSolver::Parameters params;
    params.blockSizes = blockSizes;
    options.customOptions = &params;
    dart::math::BlockedJacobiSolver solver;
    RunBenchmarkWithSolver(
        state, solver, problem, options, MakeLabel("BlockedJacobi", label));
  } else {
    state.SkipWithError("unsupported block partition sweep solver");
    return;
  }

  AddBlockPartitionSweepCounters(state, testCase, problem, blockSizes);
}

void RunApgdRestartSweepBenchmark(
    benchmark::State& state, const ApgdRestartSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(1000);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;
  options.relaxation = 1.0;

  dart::math::ApgdSolver::Parameters params;
  params.adaptiveRestart = testCase.adaptiveRestart;
  params.restartCheckInterval = testCase.restartCheckInterval;
  options.customOptions = &params;

  dart::math::ApgdSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "Apgd",
          "RestartSweep/" + std::string(getProblemFamilyName(testCase.family))
              + "/" + std::string(testCase.restartPolicyLabel)));

  state.counters["apgd_restart_sweep"] = 1.0;
  state.counters["apgd_adaptive_restart"]
      = testCase.adaptiveRestart ? 1.0 : 0.0;
  state.counters["apgd_restart_check_interval"] = testCase.restartCheckInterval;
  state.counters["apgd_relaxation"] = options.relaxation;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunTgsIterationBudgetSweepBenchmark(
    benchmark::State& state, const TgsIterationBudgetSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(testCase.maxIterations);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;

  dart::math::TgsSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "Tgs",
          "IterationBudgetSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.iterationBudgetLabel)));

  state.counters["tgs_iteration_budget_sweep"] = 1.0;
  state.counters["tgs_max_iterations"] = testCase.maxIterations;
  state.counters["tgs_relaxation"] = 1.0;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunNncgPgsIterationsSweepBenchmark(
    benchmark::State& state, const NncgPgsIterationsSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(300);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;

  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = testCase.pgsIterations;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;

  dart::math::NncgSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "NNCG",
          "PgsIterationsSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.pgsIterationsLabel)));

  state.counters["nncg_pgs_iterations_sweep"] = 1.0;
  state.counters["nncg_pgs_iterations"] = testCase.pgsIterations;
  state.counters["nncg_restart_interval"] = params.restartInterval;
  state.counters["nncg_restart_threshold"] = params.restartThreshold;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunSubspacePgsIterationsSweepBenchmark(
    benchmark::State& state, const SubspacePgsIterationsSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(300);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;

  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = testCase.pgsIterations;
  params.activeSetTolerance = 0.0;
  options.customOptions = &params;

  dart::math::SubspaceMinimizationSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "SubspaceMinimization",
          "PgsIterationsSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.pgsIterationsLabel)));

  state.counters["subspace_pgs_iterations_sweep"] = 1.0;
  state.counters["subspace_pgs_iterations"] = testCase.pgsIterations;
  state.counters["subspace_active_set_tolerance"] = params.activeSetTolerance;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunShockPropagationLayerSweepBenchmark(
    benchmark::State& state, const ShockPropagationLayerSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(100);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;

  dart::math::ShockPropagationSolver::Parameters params;
  ConfigureShockPropagationLayerSweepParameters(
      problem, testCase.profile, params);
  options.customOptions = &params;

  dart::math::ShockPropagationSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "ShockPropagation",
          "LayerSweep/" + std::string(getProblemFamilyName(testCase.family))
              + "/" + std::string(testCase.profileLabel)));

  state.counters["shock_propagation_layer_sweep"] = 1.0;
  state.counters["shock_propagation_single_layer"]
      = testCase.profile == ShockPropagationLayerProfile::SingleLayer ? 1.0
                                                                      : 0.0;
  state.counters["shock_propagation_two_layers"]
      = testCase.profile == ShockPropagationLayerProfile::TwoLayers ? 1.0 : 0.0;
  state.counters["shock_propagation_serial_layers"]
      = testCase.profile == ShockPropagationLayerProfile::SerialLayers ? 1.0
                                                                       : 0.0;
  AddShockPropagationCounters(state, params);
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunMprgpSpdCheckSweepBenchmark(
    benchmark::State& state, const MprgpSpdCheckSweepCase testCase)
{
  const auto problem = MakeStandardSpdSweepProblem(
      testCase.kind, testCase.problemSize, testCase.seed);
  auto options = MakeBenchmarkOptions(500);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance = 1e-3;

  dart::math::MprgpSolver::Parameters params;
  params.checkPositiveDefinite = testCase.checkPositiveDefinite;
  options.customOptions = &params;

  dart::math::MprgpSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "MPRGP",
          "SpdCheckSweep/" + std::string(testCase.kindLabel) + "/"
              + std::to_string(testCase.problemSize) + "/"
              + std::string(testCase.pdCheckLabel)));

  state.counters["mprgp_spd_check_sweep"] = 1.0;
  state.counters["mprgp_positive_definite_check"]
      = testCase.checkPositiveDefinite ? 1.0 : 0.0;
  state.counters["mprgp_dense_spd"]
      = testCase.kind == StandardSpdProblemKind::DenseSpd ? 1.0 : 0.0;
  state.counters["mprgp_banded_spd"]
      = testCase.kind == StandardSpdProblemKind::BandedSpd ? 1.0 : 0.0;
  state.counters["mprgp_mild_ill_conditioned"]
      = testCase.kind == StandardSpdProblemKind::MildIllConditioned ? 1.0 : 0.0;
  state.counters["mprgp_near_singular"]
      = testCase.kind == StandardSpdProblemKind::NearSingular ? 1.0 : 0.0;
  state.counters["mprgp_symmetry_tolerance"] = params.symmetryTolerance;
  state.counters["mprgp_epsilon_for_division"] = params.epsilonForDivision;
}

void RunInteriorPointPathSweepBenchmark(
    benchmark::State& state, const InteriorPointPathSweepCase testCase)
{
  const auto problem = MakeStandardSpdSweepProblem(
      testCase.kind, testCase.problemSize, testCase.seed);
  auto options = MakeBenchmarkOptions(100);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance = 1e-3;

  dart::math::InteriorPointSolver::Parameters params;
  params.sigma = testCase.sigma;
  params.stepScale = testCase.stepScale;
  options.customOptions = &params;

  dart::math::InteriorPointSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "InteriorPoint",
          "PathSweep/" + std::string(testCase.kindLabel) + "/"
              + std::to_string(testCase.problemSize) + "/"
              + std::string(testCase.sigmaLabel) + "/"
              + std::string(testCase.stepScaleLabel)));

  state.counters["interior_point_path_sweep"] = 1.0;
  state.counters["interior_point_sigma"] = params.sigma;
  state.counters["interior_point_step_scale"] = params.stepScale;
  state.counters["interior_point_dense_spd"]
      = testCase.kind == StandardSpdProblemKind::DenseSpd ? 1.0 : 0.0;
  state.counters["interior_point_banded_spd"]
      = testCase.kind == StandardSpdProblemKind::BandedSpd ? 1.0 : 0.0;
  state.counters["interior_point_mild_ill_conditioned"]
      = testCase.kind == StandardSpdProblemKind::MildIllConditioned ? 1.0 : 0.0;
  state.counters["interior_point_near_singular"]
      = testCase.kind == StandardSpdProblemKind::NearSingular ? 1.0 : 0.0;
}

void RunAdmmRhoSweepBenchmark(
    benchmark::State& state, const AdmmRhoSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(500);
  options.absoluteTolerance = 1e-5;
  options.complementarityTolerance = 1e-5;

  dart::math::AdmmSolver::Parameters params;
  params.rhoInit = testCase.rhoInit;
  params.adaptiveRho = testCase.adaptiveRho;
  options.customOptions = &params;

  dart::math::AdmmSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "Admm",
          "RhoSweep/" + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.policyLabel) + "/"
              + std::string(testCase.rhoLabel)));

  state.counters["admm_rho_sweep"] = 1.0;
  state.counters["admm_rho_init"] = testCase.rhoInit;
  state.counters["admm_mu_prox"] = params.muProx;
  state.counters["admm_adaptive_rho"] = testCase.adaptiveRho ? 1.0 : 0.0;
  state.counters["admm_fixed_rho"] = testCase.adaptiveRho ? 0.0 : 1.0;
  state.counters["admm_adaptive_rho_tolerance"] = params.adaptiveRhoTolerance;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunSapRegularizationSweepBenchmark(
    benchmark::State& state, const SapRegularizationSweepCase testCase)
{
  const auto problem
      = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
  auto options = MakeBenchmarkOptions(1000);
  options.absoluteTolerance = 1e-5;
  options.relativeTolerance = 1e-3;
  options.complementarityTolerance
      = testCase.family == BenchmarkProblemFamily::FrictionIndex ? 2e-3 : 1e-3;

  dart::math::SapSolver::Parameters params;
  params.regularization = testCase.regularization;
  params.maxLineSearchIterations = 32;
  options.customOptions = &params;

  dart::math::SapSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      problem,
      options,
      MakeLabel(
          "Sap",
          "RegularizationSweep/"
              + std::string(getProblemFamilyName(testCase.family)) + "/"
              + std::string(testCase.regularizationLabel)));

  state.counters["sap_regularization_sweep"] = 1.0;
  state.counters["sap_regularization"] = testCase.regularization;
  state.counters["sap_armijo_parameter"] = params.armijosParameter;
  state.counters["sap_backtracking_factor"] = params.backtrackingFactor;
  state.counters["sap_max_line_search_iterations"]
      = params.maxLineSearchIterations;
  if (testCase.family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = testCase.problemArg;
  }
}

void RunNewtonWarmStartBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const NewtonWarmStartMode mode)
{
  const int problemSize = static_cast<int>(state.range(0));
  const auto problem = MakeStandardActiveSetTransitionProblem(
      problemSize, 80'000u + static_cast<unsigned>(problemSize));
  NewtonWarmStartBenchmarkOptions storage;
  ConfigureNewtonWarmStartBenchmarkOptions(storage, solverEntry, mode);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(problem)) {
    state.SkipWithError(
        "Newton warm-start case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "NewtonWarmStart/" + std::string(getNewtonWarmStartModeName(mode))));

  AddNewtonWarmStartBenchmarkCounters(state, mode);
}

std::vector<LcpProblem> MakeNewtonWarmStartBatchProblems(
    const int problemSize, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  const unsigned seedBase = 81'000u + static_cast<unsigned>(problemSize);
  for (const int i : std::views::iota(0, batchSize)) {
    problems.push_back(MakeStandardActiveSetTransitionProblem(
        problemSize, seedBase + static_cast<unsigned>(997 * i)));
  }

  return problems;
}

std::vector<int> GetConcreteNewtonWarmStartProblemSizes(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::vector<int> problemSizes;
  for (const int problemSize : kNewtonWarmStartProblemSizes) {
    const auto problem = MakeStandardActiveSetTransitionProblem(
        problemSize, 80'000u + static_cast<unsigned>(problemSize));
    if (SolverSupportsConcreteProblem(solver, problem)) {
      problemSizes.push_back(problemSize);
    }
  }

  return problemSizes;
}

std::vector<std::array<int, 2>> GetConcreteNewtonWarmStartBatchArgs(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::vector<std::array<int, 2>> args;
  for (const int problemSize : kNewtonWarmStartProblemSizes) {
    const auto problems = MakeNewtonWarmStartBatchProblems(
        problemSize, kNewtonWarmStartBatchSize);
    if (SolverSupportsConcreteProblemBatch(solver, problems)) {
      args.push_back({problemSize, kNewtonWarmStartBatchSize});
    }
  }

  return args;
}

void AddNewtonWarmStartBatchArgs(
    benchmark::Benchmark* registeredBenchmark,
    const std::vector<std::array<int, 2>>& args)
{
  for (const auto [problemSize, batchSize] : args) {
    registeredBenchmark->Args({problemSize, batchSize});
  }
}

void RunNewtonWarmStartBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const NewtonWarmStartMode mode)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  const auto problems
      = MakeNewtonWarmStartBatchProblems(problemSize, batchSize);
  NewtonWarmStartBenchmarkOptions storage;
  ConfigureNewtonWarmStartBenchmarkOptions(storage, solverEntry, mode);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!std::ranges::all_of(problems, [&](const LcpProblem& problem) {
        return solver->supportsProblem(problem);
      })) {
    state.SkipWithError(
        "Newton warm-start batch case exceeds concrete solver support");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "NewtonWarmStartBatchSerial/"
              + std::string(getNewtonWarmStartModeName(mode))));
  AddNewtonWarmStartBenchmarkCounters(state, mode);
  state.counters["newton_warm_start_batch"] = 1.0;
  state.counters["batch_serial_execution"] = 1.0;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunNewtonWarmStartBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const NewtonWarmStartMode mode)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  ParallelNewtonWarmStartBatchFixture fixture(
      solverEntry,
      mode,
      MakeNewtonWarmStartBatchProblems(problemSize, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "NewtonWarmStartBatchParallel/"
              + std::string(getNewtonWarmStartModeName(mode))));
  AddNewtonWarmStartBenchmarkCounters(state, mode);
  state.counters["newton_warm_start_batch"] = 1.0;
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);
}
#endif

void ConfigureLargerActiveSetTransitionBenchmarkOptions(
    SolverBenchmarkOptions& storage,
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  storage.options.maxIterations
      = std::max(storage.options.maxIterations, 20000);
  storage.options.absoluteTolerance = 1e-4;
  storage.options.relativeTolerance
      = std::max(storage.options.relativeTolerance, 1e-2);
  storage.options.complementarityTolerance = std::max(
      storage.options.complementarityTolerance,
      isLargerActiveSetTransitionFrictionIndexCase(testCase) ? 2e-2 : 1e-2);
}

void RunLargerActiveSetTransitionBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const std::string_view labelFamily,
    const std::string_view scaleCounterName)
{
  const auto problem = MakeLargerActiveSetTransitionBenchmarkProblem(testCase);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  ConfigureLargerActiveSetTransitionBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          std::string(labelFamily) + "/"
              + std::string(getLargerActiveSetTransitionCaseName(testCase))));

  state.counters["active_set_transition"] = 1.0;
  state.counters[std::string(scaleCounterName)] = 1.0;
  if (isLargerActiveSetTransitionFrictionIndexCase(testCase)) {
    state.counters["contact_count"] = static_cast<double>(
        getLargerActiveSetTransitionContactCount(testCase));
    state.counters["coupled"] = 1.0;
    state.counters["coupling_scale"]
        = getLargerActiveSetTransitionCouplingScale(testCase);
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

std::vector<LcpProblem> MakeProductionActiveSetTransitionBatchProblems(
    const LargerActiveSetTransitionBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  for (const int i : std::views::iota(0, batchSize)) {
    problems.push_back(MakeLargerActiveSetTransitionBenchmarkProblem(
        testCase, static_cast<unsigned>(i)));
  }

  return problems;
}

void AddProductionActiveSetTransitionBatchCounters(
    benchmark::State& state,
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const int batchSize)
{
  state.counters["active_set_transition"] = 1.0;
  state.counters["production_active_set_transition_batch"] = 1.0;
  if (isLargerActiveSetTransitionFrictionIndexCase(testCase)) {
    const int contactCount = getLargerActiveSetTransitionContactCount(testCase);
    state.counters["contact_count"] = static_cast<double>(contactCount);
    state.counters["total_contact_count"]
        = static_cast<double>(contactCount * batchSize);
    state.counters["coupled"] = 1.0;
    state.counters["coupling_scale"]
        = getLargerActiveSetTransitionCouplingScale(testCase);
  }
}

void RunProductionActiveSetTransitionBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  const auto problems
      = MakeProductionActiveSetTransitionBatchProblems(testCase, batchSize);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());
  ConfigureLargerActiveSetTransitionBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "ProductionActiveSetTransitionBatchSerial/"
              + std::string(getLargerActiveSetTransitionCaseName(testCase))));
  AddProductionActiveSetTransitionBatchCounters(state, testCase, batchSize);
  state.counters["batch_serial_execution"] = 1.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunProductionActiveSetTransitionBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  ParallelBatchFixture fixture(
      solverEntry,
      MakeProductionActiveSetTransitionBatchProblems(testCase, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  ConfigureLargerActiveSetTransitionBenchmarkOptions(fixture.storage, testCase);

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "ProductionActiveSetTransitionBatchParallel/"
              + std::string(getLargerActiveSetTransitionCaseName(testCase))));
  AddProductionActiveSetTransitionBatchCounters(state, testCase, batchSize);
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
}
#endif

void ConfigureMildIllConditionedBenchmarkOptions(
    SolverBenchmarkOptions& storage,
    const dart::test::LcpSolverManifestEntry& solver,
    const MildIllConditionedBenchmarkCase testCase)
{
  storage.options.maxIterations
      = std::max(storage.options.maxIterations, 20000);
  storage.options.absoluteTolerance = 1e-4;
  storage.options.relativeTolerance = 5e-3;
  storage.options.complementarityTolerance
      = isMildIllConditionedFrictionIndexCase(testCase) ? 2e-2 : 5e-3;
  storage.options.earlyTermination = true;

  if (solver.name == "BoxedSemiSmoothNewton"
      && isMildIllConditionedCoupledFrictionIndexCase(testCase)
      && getMildIllConditionedCouplingScale(testCase) > 8.0) {
    storage.boxedSsnParams.maxLineSearchSteps = 50;
    storage.boxedSsnParams.stepReduction = 0.8;
    storage.boxedSsnParams.jacobianRegularization = 1e-8;
    storage.options.customOptions = &storage.boxedSsnParams;
    storage.hasBoxedSsnParams = true;
  }
}

void RunMildIllConditionedBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    MildIllConditionedBenchmarkCase testCase)
{
  const auto problem = MakeMildIllConditionedBenchmarkProblem(testCase);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  ConfigureMildIllConditionedBenchmarkOptions(storage, solverEntry, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "MildIllConditioned/"
              + std::string(getMildIllConditionedCaseName(testCase))));

  state.counters["mildly_ill_conditioned"] = 1.0;
  if (isMildIllConditionedFrictionIndexCase(testCase)) {
    state.counters["contact_count"]
        = static_cast<double>(getMildIllConditionedContactCount(testCase));
  }
  if (isMildIllConditionedCoupledFrictionIndexCase(testCase)) {
    state.counters["coupled"] = 1.0;
    state.counters["coupling_scale"]
        = getMildIllConditionedCouplingScale(testCase);
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
  if (storage.hasBoxedSsnParams) {
    AddBoxedSsnCounters(state, storage.boxedSsnParams);
  }
}

std::vector<LcpProblem> MakeMildIllConditionedBatchProblems(
    const MildIllConditionedBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  const int contactCount = getMildIllConditionedContactCount(testCase);
  const double couplingScale = getMildIllConditionedCouplingScale(testCase);
  unsigned seedBase = 24'032u;
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::Standard32:
      seedBase = 24'032u;
      break;
    case MildIllConditionedBenchmarkCase::Boxed16:
      seedBase = 25'016u;
      break;
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
      seedBase = 26'008u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
      seedBase = 27'006u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
      seedBase = 27'008u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
      seedBase = 27'012u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
      seedBase = 27'016u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
      seedBase = 27'024u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
      seedBase = 27'032u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
      seedBase = 27'048u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
      seedBase = 27'064u;
      break;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
      seedBase = 27'096u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
      seedBase = 28'006u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
      seedBase = 28'008u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
      seedBase = 28'012u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      seedBase = 28'016u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      seedBase = 28'024u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
      seedBase = 28'032u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
      seedBase = 28'048u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
      seedBase = 28'064u;
      break;
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
      seedBase = 28'096u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
      seedBase = 29'006u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
      seedBase = 29'008u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
      seedBase = 29'012u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      seedBase = 29'016u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      seedBase = 29'024u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
      seedBase = 29'032u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
      seedBase = 29'048u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
      seedBase = 29'064u;
      break;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      seedBase = 29'096u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
      seedBase = 30'006u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
      seedBase = 30'008u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
      seedBase = 30'012u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
      seedBase = 30'016u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
      seedBase = 30'024u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
      seedBase = 30'032u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
      seedBase = 30'048u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
      seedBase = 30'064u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
      seedBase = 30'096u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
      seedBase = 30'128u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
      seedBase = 30'192u;
      break;
    case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
      seedBase = 30'256u;
      break;
    default:
      break;
  }

  for (const int i : std::views::iota(0, batchSize)) {
    const unsigned seed = seedBase + static_cast<unsigned>(i);
    switch (testCase) {
      case MildIllConditionedBenchmarkCase::Standard32:
        problems.push_back(MakeMildIllConditionedStandardProblem(32, seed));
        break;
      case MildIllConditionedBenchmarkCase::Boxed16:
        problems.push_back(MakeMildIllConditionedBoxedProblem(16, seed));
        break;
      case MildIllConditionedBenchmarkCase::FrictionIndex8:
        problems.push_back(
            MakeMildIllConditionedFrictionIndexProblem(8, seed, false));
        break;
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex32:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex48:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex64:
      case MildIllConditionedBenchmarkCase::CoupledFrictionIndex96:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64:
      case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64:
      case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192:
      case MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256:
        problems.push_back(MakeMildIllConditionedFrictionIndexProblem(
            contactCount, seed, true, couplingScale));
        break;
    }
  }

  return problems;
}

void AddMildIllConditionedBatchCounters(
    benchmark::State& state,
    const MildIllConditionedBenchmarkCase testCase,
    const int batchSize)
{
  const int contactCount = getMildIllConditionedContactCount(testCase);
  state.counters["mildly_ill_conditioned"] = 1.0;
  state.counters["mildly_ill_conditioned_batch"] = 1.0;
  if (isMildIllConditionedFrictionIndexCase(testCase)) {
    state.counters["contact_count"] = static_cast<double>(contactCount);
    state.counters["total_contact_count"]
        = static_cast<double>(contactCount * batchSize);
  }
  if (isMildIllConditionedCoupledFrictionIndexCase(testCase)) {
    state.counters["coupled"] = 1.0;
    state.counters["coupling_scale"]
        = getMildIllConditionedCouplingScale(testCase);
  }
}

void RunMildIllConditionedBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const MildIllConditionedBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  const auto problems
      = MakeMildIllConditionedBatchProblems(testCase, batchSize);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());
  ConfigureMildIllConditionedBenchmarkOptions(storage, solverEntry, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "MildIllConditionedBatchSerial/"
              + std::string(getMildIllConditionedCaseName(testCase))));
  AddMildIllConditionedBatchCounters(state, testCase, batchSize);
  state.counters["batch_serial_execution"] = 1.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
  if (storage.hasBoxedSsnParams) {
    AddBoxedSsnCounters(state, storage.boxedSsnParams);
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunMildIllConditionedBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const MildIllConditionedBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  ParallelBatchFixture fixture(
      solverEntry, MakeMildIllConditionedBatchProblems(testCase, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  ConfigureMildIllConditionedBenchmarkOptions(
      fixture.storage, solverEntry, testCase);

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "MildIllConditionedBatchParallel/"
              + std::string(getMildIllConditionedCaseName(testCase))));
  AddMildIllConditionedBatchCounters(state, testCase, batchSize);
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
  if (fixture.storage.hasBoxedSsnParams) {
    AddBoxedSsnCounters(state, fixture.storage.boxedSsnParams);
  }
}
#endif

void ConfigureNearSingularBenchmarkOptions(
    SolverBenchmarkOptions& storage, const NearSingularBenchmarkCase testCase)
{
  storage.options.maxIterations
      = std::max(storage.options.maxIterations, 20000);
  storage.options.absoluteTolerance = 1e-4;
  storage.options.relativeTolerance = 1e-2;
  storage.options.complementarityTolerance
      = isNearSingularFrictionIndexCase(testCase) ? 2e-2 : 1e-2;
  storage.options.earlyTermination = true;
}

void RunNearSingularBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    NearSingularBenchmarkCase testCase)
{
  const auto problem = MakeNearSingularBenchmarkProblem(testCase);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  ConfigureNearSingularBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "NearSingular/" + std::string(getNearSingularCaseName(testCase))));

  state.counters["near_singular"] = 1.0;
  if (isNearSingularFrictionIndexCase(testCase)) {
    state.counters["contact_count"]
        = static_cast<double>(getNearSingularContactCount(testCase));
    state.counters["coupled"] = 1.0;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

void AddNearSingularBatchCounters(
    benchmark::State& state,
    const NearSingularBenchmarkCase testCase,
    const int batchSize)
{
  state.counters["near_singular"] = 1.0;
  state.counters["near_singular_batch"] = 1.0;
  if (isNearSingularFrictionIndexCase(testCase)) {
    const int contactCount = getNearSingularContactCount(testCase);
    state.counters["contact_count"] = static_cast<double>(contactCount);
    state.counters["total_contact_count"]
        = static_cast<double>(contactCount * batchSize);
    state.counters["coupled"] = 1.0;
  }
}

void RunNearSingularBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const NearSingularBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  const auto problems = MakeNearSingularBatchProblems(testCase, batchSize);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());
  ConfigureNearSingularBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "NearSingularBatchSerial/"
              + std::string(getNearSingularCaseName(testCase))));
  AddNearSingularBatchCounters(state, testCase, batchSize);
  state.counters["batch_serial_execution"] = 1.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunNearSingularBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const NearSingularBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  ParallelBatchFixture fixture(
      solverEntry, MakeNearSingularBatchProblems(testCase, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  ConfigureNearSingularBenchmarkOptions(fixture.storage, testCase);

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "NearSingularBatchParallel/"
              + std::string(getNearSingularCaseName(testCase))));
  AddNearSingularBatchCounters(state, testCase, batchSize);
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
}
#endif

void ConfigureSingularDegenerateBenchmarkOptions(
    SolverBenchmarkOptions& storage,
    const SingularDegenerateBenchmarkCase testCase)
{
  storage.options.maxIterations
      = std::max(storage.options.maxIterations, 20000);
  storage.options.absoluteTolerance = 1e-4;
  storage.options.relativeTolerance = 1e-2;
  storage.options.complementarityTolerance
      = isSingularDegenerateFrictionIndexCase(testCase) ? 2e-2 : 1e-2;
  storage.options.earlyTermination = true;
}

void RunSingularDegenerateBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    SingularDegenerateBenchmarkCase testCase)
{
  const auto problem = MakeSingularDegenerateBenchmarkProblem(testCase);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  ConfigureSingularDegenerateBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "SingularDegenerate/"
              + std::string(getSingularDegenerateCaseName(testCase))));

  state.counters["singular_degenerate"] = 1.0;
  state.counters["rank_deficient"] = 1.0;
  if (isSingularDegenerateFrictionIndexCase(testCase)) {
    state.counters["contact_count"]
        = static_cast<double>(getSingularDegenerateContactCount(testCase));
    state.counters["coupled"] = 1.0;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

void AddSingularDegenerateFrictionIndexBatchCounters(
    benchmark::State& state,
    const SingularDegenerateBenchmarkCase testCase,
    const int batchSize)
{
  const int contactCount = getSingularDegenerateContactCount(testCase);
  state.counters["singular_degenerate"] = 1.0;
  state.counters["singular_degenerate_batch"] = 1.0;
  state.counters["rank_deficient"] = 1.0;
  state.counters["contact_count"] = static_cast<double>(contactCount);
  state.counters["total_contact_count"]
      = static_cast<double>(contactCount * batchSize);
  state.counters["coupled"] = 1.0;
}

void AddSingularDegenerateStandardBoxedBatchCounters(
    benchmark::State& state,
    const SingularDegenerateBenchmarkCase testCase,
    const int batchSize)
{
  const auto problemSize = static_cast<double>(
      MakeSingularDegenerateBenchmarkProblem(testCase).b.size());
  state.counters["singular_degenerate"] = 1.0;
  state.counters["singular_degenerate_batch"] = 1.0;
  state.counters["singular_degenerate_standard_boxed_batch"] = 1.0;
  state.counters["rank_deficient"] = 1.0;
  state.counters["total_problem_size"]
      = problemSize * static_cast<double>(batchSize);
}

void RunSingularDegenerateFrictionIndexBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const SingularDegenerateBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  const auto problems
      = MakeSingularDegenerateFrictionIndexBatchProblems(testCase, batchSize);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());
  ConfigureSingularDegenerateBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "SingularDegenerateFrictionIndexBatchSerial/"
              + std::string(getSingularDegenerateCaseName(testCase))));
  AddSingularDegenerateFrictionIndexBatchCounters(state, testCase, batchSize);
  state.counters["batch_serial_execution"] = 1.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunSingularDegenerateFrictionIndexBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const SingularDegenerateBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  ParallelBatchFixture fixture(
      solverEntry,
      MakeSingularDegenerateFrictionIndexBatchProblems(testCase, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  ConfigureSingularDegenerateBenchmarkOptions(fixture.storage, testCase);

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "SingularDegenerateFrictionIndexBatchParallel/"
              + std::string(getSingularDegenerateCaseName(testCase))));
  AddSingularDegenerateFrictionIndexBatchCounters(state, testCase, batchSize);
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
}
#endif

void RunSingularDegenerateStandardBoxedBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const SingularDegenerateBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  const auto problems
      = MakeSingularDegenerateStandardBoxedBatchProblems(testCase, batchSize);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());
  ConfigureSingularDegenerateBenchmarkOptions(storage, testCase);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "SingularDegenerateStandardBoxedBatchSerial/"
              + std::string(getSingularDegenerateCaseName(testCase))));
  AddSingularDegenerateStandardBoxedBatchCounters(state, testCase, batchSize);
  state.counters["batch_serial_execution"] = 1.0;
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunSingularDegenerateStandardBoxedBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const SingularDegenerateBenchmarkCase testCase)
{
  const int batchSize = static_cast<int>(state.range(0));
  ParallelBatchFixture fixture(
      solverEntry,
      MakeSingularDegenerateStandardBoxedBatchProblems(testCase, batchSize));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  ConfigureSingularDegenerateBenchmarkOptions(fixture.storage, testCase);

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "SingularDegenerateStandardBoxedBatchParallel/"
              + std::string(getSingularDegenerateCaseName(testCase))));
  AddSingularDegenerateStandardBoxedBatchCounters(state, testCase, batchSize);
  state.counters["batch_parallel_execution"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
}
#endif

void RunManifestBatchBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const int problemArg = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  const auto problems
      = MakeBenchmarkProblemBatch(family, problemArg, batchSize);

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "BatchSerial/" + std::string(getProblemFamilyName(family))));

  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
  }
}

void RunGroupedBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const int variantsPerProblemArg = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto batch
      = MakeGroupedBenchmarkBatch(family, variantsPerProblemArg, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  const auto problems = FlattenGroupedBenchmarkBatch(*batch);

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problems.front());

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, problems, storage.options);
  AddGroupedBatchBenchmarkCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          "GroupedBatchSerial/" + std::string(getProblemFamilyName(family))));
  state.counters["grouped_batch_serial"] = 1.0;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunWorldContactBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const int contactCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixture
      = MakeWorldContactBenchmarkProblem(contactCount, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, fixture->problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      fixture->problem,
      storage.options,
      MakeLabel(std::string(solverEntry.name), "WorldContact/FrictionIndex"));

  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["body_count"] = static_cast<double>(fixture->bodyCount);
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
  }
}

void RunWorldBoxContactBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const int boxCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixture
      = MakeWorldBoxContactBenchmarkProblem(errorMessage, 0, boxCount);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, fixture->problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(fixture->problem)) {
    state.SkipWithError(
        "World box contact case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      fixture->problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name), "WorldBoxContact/FrictionIndex"));

  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["body_count"] = static_cast<double>(fixture->bodyCount);
  state.counters["box_count"] = static_cast<double>(boxCount);
  state.counters["dense_box_contact"] = 1.0;
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
  }
}

void RunWorldStackContactBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const int sphereCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixture
      = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, fixture->problem);
  if (solverEntry.name == "Pgs") {
    // The 12-sphere coupled stack converges in 302 PGS iterations; the
    // default 100-iteration cap leaves the 8-sphere row outside the LCP
    // contract. The 24-/32-sphere rows need 886/1344 iterations.
    storage.options.maxIterations = 512;
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "Jacobi") {
    // The 12-sphere coupled stack converges in 452 Jacobi iterations; the
    // default 100-iteration cap leaves the 8-sphere row outside the same LCP
    // contract. The 14-/15-/16-/24-/32-sphere rows need 568/628/688/1184/1672
    // iterations.
    storage.options.maxIterations = 512;
    if (sphereCount >= 14) {
      storage.options.maxIterations = 1024;
    }
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "BlockedJacobi") {
    // The 12-sphere coupled stack converges in 452 blocked Jacobi iterations;
    // the default 100-iteration cap leaves the 8-sphere row outside the same
    // LCP contract. The 14-/15-/16-/24-/32-sphere rows need
    // 568/628/688/1184/1672
    // iterations.
    storage.options.maxIterations = 512;
    if (sphereCount >= 14) {
      storage.options.maxIterations = 1024;
    }
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "NNCG") {
    // Coupled stack contacts need a stronger PGS preconditioner than the
    // generated math fixtures; the 12-sphere row needs a stronger
    // preconditioner than the smaller coupled-stack rows, and the 24-/32-sphere
    // rows need 160 PGS preconditioner iterations.
    storage.nncgParams.pgsIterations = 20;
    if (sphereCount >= 11) {
      storage.options.maxIterations = 512;
    }
    if (sphereCount >= 12) {
      storage.nncgParams.pgsIterations = 40;
    }
    if (sphereCount >= 24) {
      storage.options.maxIterations = 4096;
      storage.nncgParams.pgsIterations = 160;
    }
  } else if (solverEntry.name == "SymmetricPsor") {
    // The 11-/12-/24-/32-sphere rows converge past the default 100-iteration
    // cap.
    if (sphereCount >= 11) {
      storage.options.maxIterations = 512;
    }
  } else if (solverEntry.name == "BGS") {
    // The 12-/24-/32-sphere rows need 219/573/810 block Gauss-Seidel
    // iterations.
    if (sphereCount >= 11) {
      storage.options.maxIterations = 512;
    }
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "RedBlackGaussSeidel") {
    // The 12-sphere coupled stack converges in 259 two-color iterations; 128
    // iterations leave the 8-sphere row just outside the same LCP contract.
    // The 24-/32-sphere rows need 727/1071 iterations.
    storage.options.maxIterations = 512;
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "ShockPropagation") {
    // The 12-/24-/32-sphere coupled stacks need 219/573/810
    // shock-propagation sweeps.
    storage.options.maxIterations = 512;
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  } else if (solverEntry.name == "Tgs") {
    // The 12-/24-/32-sphere rows need 302/886/1344 temporal Gauss-Seidel
    // iterations.
    if (sphereCount >= 11) {
      storage.options.maxIterations = 512;
    }
    if (sphereCount >= 24) {
      storage.options.maxIterations = 2048;
    }
  }

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      fixture->problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name), "WorldStackContact/FrictionIndex"));

  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["body_count"] = static_cast<double>(fixture->bodyCount);
  state.counters["sphere_count"] = static_cast<double>(sphereCount);
  if (solverEntry.name == "Pgs") {
    AddPgsCounters(state, storage.options);
  }
  if (solverEntry.name == "SymmetricPsor") {
    AddSymmetricPsorCounters(state, storage.options);
  }
  if (solverEntry.name == "BGS") {
    AddBgsCounters(state, storage.options);
  }
  if (solverEntry.name == "Jacobi") {
    AddJacobiCounters(state, storage.options);
  }
  if (solverEntry.name == "BlockedJacobi") {
    AddBlockedJacobiCounters(state, storage.options);
  }
  if (storage.hasNncgParams) {
    AddNncgCounters(state, storage.nncgParams);
  }
  if (solverEntry.name == "RedBlackGaussSeidel") {
    AddRedBlackGaussSeidelCounters(state, storage.options);
  }
  if (solverEntry.name == "Tgs") {
    AddTgsCounters(state, storage.options);
  }
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
    AddShockPropagationOptionCounters(state, storage.options);
  }
}

void RunArticulatedUnifiedContactBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    ArticulatedContactBenchmarkCase benchmarkCase)
{
  const int contactCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixture = MakeArticulatedUnifiedContactBenchmarkProblem(
      benchmarkCase, contactCount, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, fixture->problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(fixture->problem)) {
    state.SkipWithError(
        "Articulated unified contact case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      fixture->problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "ArticulatedUnifiedContact/FrictionIndex/"
              + std::string(
                  getArticulatedContactBenchmarkCaseName(benchmarkCase))));

  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["multibody_count"] = static_cast<double>(
      benchmarkCase == ArticulatedContactBenchmarkCase::CrossLinkImpact
          ? 2 * fixture->contactCount
          : fixture->contactCount);
  state.counters["articulated_unified_contact"] = 1.0;
  state.counters["articulated_ground_contact"]
      = benchmarkCase == ArticulatedContactBenchmarkCase::Ground ? 1.0 : 0.0;
  state.counters["articulated_rigid_impact_contact"]
      = benchmarkCase == ArticulatedContactBenchmarkCase::RigidImpact ? 1.0
                                                                      : 0.0;
  state.counters["articulated_cross_link_contact"]
      = benchmarkCase == ArticulatedContactBenchmarkCase::CrossLinkImpact ? 1.0
                                                                          : 0.0;
  if (benchmarkCase == ArticulatedContactBenchmarkCase::RigidImpact) {
    state.counters["dynamic_rigid_body_count"]
        = static_cast<double>(fixture->bodyCount);
  }
  if (storage.hasSapParams) {
    AddSapCounters(state, storage.sapParams);
  }
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
  }
}

void RunStaggeringContactPipelineSweepBenchmark(
    benchmark::State& state, const StaggeringContactPipelineSweepCase testCase)
{
  std::string errorMessage;
  const auto fixture
      = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  auto options = MakeBenchmarkOptions(100);
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;

  dart::math::StaggeringSolver solver;
  RunBenchmarkWithSolver(
      state,
      solver,
      fixture->problem,
      options,
      MakeLabel(
          "Staggering",
          "ContactPipelineSweep/" + std::string(testCase.caseLabel)));

  const auto normalRows = (fixture->problem.findex.array() < 0).count();
  const auto frictionRows = fixture->problem.findex.size() - normalRows;
  state.counters["staggering_contact_pipeline_sweep"] = 1.0;
  state.counters["staggering_normal_friction_split"] = 1.0;
  state.counters["staggering_normal_row_count"]
      = static_cast<double>(normalRows);
  state.counters["staggering_friction_row_count"]
      = static_cast<double>(frictionRows);
  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["body_count"] = static_cast<double>(fixture->bodyCount);
  state.counters["staggering_world_separated_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
  state.counters["staggering_world_stack_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldStack ? 1.0 : 0.0;
  state.counters["staggering_articulated_unified_contact"]
      = (testCase.kind == StaggeringContactPipelineKind::ArticulatedGround
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedRigidImpact
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedCrossLinkImpact)
            ? 1.0
            : 0.0;
  state.counters["staggering_articulated_ground_contact"]
      = testCase.kind == StaggeringContactPipelineKind::ArticulatedGround ? 1.0
                                                                          : 0.0;
  state.counters["staggering_articulated_rigid_impact_contact"]
      = testCase.kind == StaggeringContactPipelineKind::ArticulatedRigidImpact
            ? 1.0
            : 0.0;
  state.counters["staggering_articulated_cross_link_contact"]
      = testCase.kind
                == StaggeringContactPipelineKind::ArticulatedCrossLinkImpact
            ? 1.0
            : 0.0;
  state.counters["staggering_coupled_contact_pipeline"]
      = testCase.kind != StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
}

void RunContactSolverComparisonSweepBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const StaggeringContactPipelineSweepCase testCase)
{
  std::string errorMessage;
  const auto fixture
      = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, fixture->problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(fixture->problem)) {
    state.SkipWithError(
        "Contact solver comparison case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      fixture->problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "ContactSolverComparisonSweep/" + std::string(testCase.caseLabel)));

  const auto normalRows = (fixture->problem.findex.array() < 0).count();
  const auto frictionRows = fixture->problem.findex.size() - normalRows;
  state.counters["contact_solver_comparison_sweep"] = 1.0;
  state.counters["contact_solver_comparison_admm"]
      = solverEntry.name == "Admm" ? 1.0 : 0.0;
  state.counters["contact_solver_comparison_sap"]
      = solverEntry.name == "Sap" ? 1.0 : 0.0;
  state.counters["contact_solver_comparison_boxed_ssn"]
      = solverEntry.name == "BoxedSemiSmoothNewton" ? 1.0 : 0.0;
  state.counters["contact_count"] = static_cast<double>(fixture->contactCount);
  state.counters["body_count"] = static_cast<double>(fixture->bodyCount);
  state.counters["normal_row_count"] = static_cast<double>(normalRows);
  state.counters["friction_row_count"] = static_cast<double>(frictionRows);
  state.counters["contact_comparison_world_separated_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
  state.counters["contact_comparison_world_stack_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldStack ? 1.0 : 0.0;
  state.counters["contact_comparison_articulated_unified_contact"]
      = (testCase.kind == StaggeringContactPipelineKind::ArticulatedGround
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedRigidImpact
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedCrossLinkImpact)
            ? 1.0
            : 0.0;
  state.counters["contact_comparison_articulated_ground_contact"]
      = testCase.kind == StaggeringContactPipelineKind::ArticulatedGround ? 1.0
                                                                          : 0.0;
  state.counters["contact_comparison_articulated_rigid_impact_contact"]
      = testCase.kind == StaggeringContactPipelineKind::ArticulatedRigidImpact
            ? 1.0
            : 0.0;
  state.counters["contact_comparison_articulated_cross_link_contact"]
      = testCase.kind
                == StaggeringContactPipelineKind::ArticulatedCrossLinkImpact
            ? 1.0
            : 0.0;
  state.counters["contact_comparison_coupled_contact_fixture"]
      = testCase.kind != StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
}

void RunContactNormalStandardSweepBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    const StaggeringContactPipelineSweepCase testCase)
{
  std::string errorMessage;
  const auto fixture
      = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
  if (!fixture.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto problem = MakeContactNormalStandardProblem(*fixture, errorMessage);
  if (!problem.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, *problem);

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!solver->supportsProblem(*problem)) {
    state.SkipWithError(
        "Contact-normal standard case exceeds concrete solver support");
    return;
  }

  RunBenchmarkWithSolver(
      state,
      *solver,
      *problem,
      storage.options,
      MakeLabel(
          std::string(solverEntry.name),
          "ContactNormalStandardSweep/" + std::string(testCase.caseLabel)));

  state.counters["contact_normal_standard_sweep"] = 1.0;
  state.counters["contact_normal_standard_subproblem"] = 1.0;
  state.counters["source_contact_count"]
      = static_cast<double>(fixture->contactCount);
  state.counters["normal_row_count"]
      = static_cast<double>(fixture->contactCount);
  state.counters["source_problem_size"]
      = static_cast<double>(fixture->problem.b.size());
  state.counters["contact_normal_direct_enumeration"]
      = solverEntry.name == "Direct" ? 1.0 : 0.0;
  state.counters["contact_normal_direct_no_fallback"]
      = solverEntry.name == "Direct" && problem->b.size() <= 3 ? 1.0 : 0.0;
  state.counters["contact_normal_dantzig_baseline"]
      = solverEntry.name == "Dantzig" ? 1.0 : 0.0;
  state.counters["contact_normal_world_separated_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
  state.counters["contact_normal_world_stack_contact"]
      = testCase.kind == StaggeringContactPipelineKind::WorldStack ? 1.0 : 0.0;
  state.counters["contact_normal_articulated_unified_contact"]
      = (testCase.kind == StaggeringContactPipelineKind::ArticulatedGround
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedRigidImpact
         || testCase.kind
                == StaggeringContactPipelineKind::ArticulatedCrossLinkImpact)
            ? 1.0
            : 0.0;
  state.counters["contact_normal_coupled_contact_fixture"]
      = testCase.kind != StaggeringContactPipelineKind::WorldSeparated ? 1.0
                                                                       : 0.0;
}

void RunWorldContactBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    WorldContactBatchKind batchKind = WorldContactBatchKind::Baseline)
{
  std::string errorMessage;
  const auto batch = MakeWorldContactBenchmarkBatch(errorMessage, batchKind);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(
      storage, solverEntry, batch->problems.front());
  if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    ConfigureContactPipeline32BatchOptions(storage, solverEntry);
  }

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, batch->problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, batch->problems, storage.options);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          batchKind == WorldContactBatchKind::ContactPipeline32
              ? "WorldContactPipeline32BatchSerial/FrictionIndex"
              : (batchKind == WorldContactBatchKind::StressStack
                     ? "WorldContactStressBatchSerial/FrictionIndex"
                     : "WorldContactBatchSerial/FrictionIndex")));
  if (batchKind == WorldContactBatchKind::StressStack) {
    state.counters["stress_stack_contact_batch"] = 1.0;
    state.counters["separated_contact_shape_count"] = 3.0;
    state.counters["stack_contact_shape_count"] = 4.0;
  } else if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    state.counters["contact_pipeline_32_batch"] = 1.0;
    state.counters["contact_count_per_problem"] = 32.0;
    state.counters["separated_contact_shape_count"] = 1.0;
    state.counters["stack_contact_shape_count"] = 1.0;
    state.counters["articulated_contact_shape_count"] = 3.0;
    AddContactPipeline32BatchSolverCounters(state, storage, solverEntry);
  }

  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, batch->totalContactCount, batch->totalProblemSize);
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
  }
}

void RunWorldContactBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    WorldContactBatchKind batchKind = WorldContactBatchKind::Baseline)
{
  std::string errorMessage;
  const auto batch = MakeWorldContactBenchmarkBatch(errorMessage, batchKind);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  ParallelBatchFixture fixture(solverEntry, batch->problems);
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }
  if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    ConfigureContactPipeline32BatchOptions(fixture.storage, solverEntry);
  }

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          batchKind == WorldContactBatchKind::ContactPipeline32
              ? "WorldContactPipeline32BatchParallel/FrictionIndex"
              : (batchKind == WorldContactBatchKind::StressStack
                     ? "WorldContactStressBatchParallel/FrictionIndex"
                     : "WorldContactBatchParallel/FrictionIndex")));
  if (batchKind == WorldContactBatchKind::StressStack) {
    state.counters["stress_stack_contact_batch"] = 1.0;
    state.counters["separated_contact_shape_count"] = 3.0;
    state.counters["stack_contact_shape_count"] = 4.0;
  } else if (batchKind == WorldContactBatchKind::ContactPipeline32) {
    state.counters["contact_pipeline_32_batch"] = 1.0;
    state.counters["contact_count_per_problem"] = 32.0;
    state.counters["separated_contact_shape_count"] = 1.0;
    state.counters["stack_contact_shape_count"] = 1.0;
    state.counters["articulated_contact_shape_count"] = 3.0;
    AddContactPipeline32BatchSolverCounters(
        state, fixture.storage, solverEntry);
  }
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (fixture.storage.hasShockPropagationParams) {
    if (fixture.storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, batch->totalContactCount, batch->totalProblemSize);
    } else {
      AddShockPropagationCounters(
          state, fixture.storage.shockPropagationParams);
    }
  }
}

void AddWorldBoxContactBatchShapeCounters(
    benchmark::State& state, int boxCount, int batchSize, bool parallel)
{
  state.counters["dense_box_contact"] = 1.0;
  state.counters["dense_box_contact_batch"] = 1.0;
  state.counters["dense_box_contact_batch_parallel"] = parallel ? 1.0 : 0.0;
  state.counters["box_count"] = static_cast<double>(boxCount);
  state.counters["total_box_count"] = static_cast<double>(boxCount * batchSize);
  state.counters["contact_count"] = static_cast<double>(4 * boxCount);
  state.counters["problem_size"] = static_cast<double>(12 * boxCount);
}

void RunWorldBoxContactBatchSerialBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const int boxCount = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  std::string errorMessage;
  const auto batch
      = MakeWorldBoxContactBenchmarkBatch(boxCount, batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(
      storage, solverEntry, batch->problems.front());

  const auto solver = solverEntry.create();
  if (solver == nullptr) {
    state.SkipWithError("LCP solver factory returned null");
    return;
  }
  if (!std::ranges::all_of(batch->problems, [&](const LcpProblem& problem) {
        return solver->supportsProblem(problem);
      })) {
    state.SkipWithError(
        "World box contact batch case exceeds concrete solver support");
    return;
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    counters = RunBatchWithSolver(*solver, batch->problems, storage.options);
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  counters = RunBatchWithSolver(*solver, batch->problems, storage.options);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          "WorldBoxContactBatchSerial/FrictionIndex"));
  AddWorldBoxContactBatchShapeCounters(state, boxCount, batchSize, false);
}

void RunWorldBoxContactBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry)
{
  const int boxCount = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  std::string errorMessage;
  const auto batch
      = MakeWorldBoxContactBenchmarkBatch(boxCount, batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  ParallelBatchFixture fixture(solverEntry, batch->problems);
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          "WorldBoxContactBatchParallel/FrictionIndex"));
  AddWorldBoxContactBatchShapeCounters(state, boxCount, batchSize, true);
  AddParallelExecutionCounters(state, profile, fixture.graph);
}

static void BM_LcpWorldContactAssembly_BoxedLcp(benchmark::State& state)
{
  const int contactCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixtureOpt
      = MakeWorldContactBenchmarkProblem(contactCount, errorMessage);
  if (!fixtureOpt.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  const WorldContactBenchmarkProblem& fixture = *fixtureOpt;

  dart::test::LcpCheckResult check;
  for (auto _ : state) {
    auto problem = MakeWorldContactBenchmarkProblem(contactCount, errorMessage);
    if (!problem.has_value()) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    check = dart::test::CheckLcpSolution(
        problem->problem,
        problem->referenceSolution,
        MakeBenchmarkOptions(100));
    benchmark::DoNotOptimize(check.residual);
  }

  check = dart::test::CheckLcpSolution(
      fixture.problem, fixture.referenceSolution, MakeBenchmarkOptions(100));
  state.counters["contact_count"] = static_cast<double>(fixture.contactCount);
  state.counters["body_count"] = static_cast<double>(fixture.bodyCount);
  state.counters["problem_size"] = fixture.problem.b.size();
  state.counters["residual"] = check.residual;
  state.counters["complementarity"] = check.complementarity;
  state.counters["bound_violation"] = check.boundViolation;
  state.counters["contract_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * contactCount);
  state.SetLabel("BoxedLcpContact/WorldAssembly");
}

static void BM_LcpWorldStackContactAssembly_BoxedLcp(benchmark::State& state)
{
  const int sphereCount = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto fixtureOpt
      = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage);
  if (!fixtureOpt.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  const WorldContactBenchmarkProblem& fixture = *fixtureOpt;

  dart::test::LcpCheckResult check;
  for (auto _ : state) {
    auto problem
        = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage);
    if (!problem.has_value()) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    check = dart::test::CheckLcpSolution(
        problem->problem,
        problem->referenceSolution,
        MakeBenchmarkOptions(100));
    benchmark::DoNotOptimize(check.residual);
  }

  check = dart::test::CheckLcpSolution(
      fixture.problem, fixture.referenceSolution, MakeBenchmarkOptions(100));
  state.counters["contact_count"] = static_cast<double>(fixture.contactCount);
  state.counters["body_count"] = static_cast<double>(fixture.bodyCount);
  state.counters["sphere_count"] = static_cast<double>(sphereCount);
  state.counters["problem_size"] = fixture.problem.b.size();
  state.counters["residual"] = check.residual;
  state.counters["complementarity"] = check.complementarity;
  state.counters["bound_violation"] = check.boundViolation;
  state.counters["contract_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * fixture.contactCount);
  state.SetLabel("BoxedLcpContact/WorldStackAssembly");
}

static void BM_LcpWorldStackStep_BoxedLcp(benchmark::State& state)
{
  const int sphereCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldStackStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldStackStepBenchmarkWorld(sphereCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->enterSimulationMode();
    world->step(stepCount);
    check = CheckWorldStackStepInvariants(*world, sphereCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldStackStepBenchmarkWorld(sphereCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->enterSimulationMode();
  world->step(stepCount);
  check = CheckWorldStackStepInvariants(*world, sphereCount);

  state.counters["sphere_count"] = static_cast<double>(sphereCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["time_step"] = world->getTimeStep();
  state.counters["body_count"] = static_cast<double>(sphereCount);
  state.counters["contact_count"] = static_cast<double>(sphereCount);
  state.counters["min_spacing"] = check.minSpacing;
  state.counters["max_vertical_speed"] = check.maxVerticalSpeed;
  state.counters["max_lateral_position"] = check.maxLateralPosition;
  state.counters["max_lateral_speed"] = check.maxLateralSpeed;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount);
  state.SetLabel("BoxedLcpContact/WorldStackStep");
}

static void BM_LcpWorldSeparatedStep_BoxedLcp(benchmark::State& state)
{
  const int sphereCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldSeparatedStepCheck check;
  for (auto _ : state) {
    auto world
        = MakeWorldSeparatedStepBenchmarkWorld(sphereCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->enterSimulationMode();
    world->step(stepCount);
    check = CheckWorldSeparatedStepInvariants(*world, sphereCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldSeparatedStepBenchmarkWorld(sphereCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->enterSimulationMode();
  world->step(stepCount);
  check = CheckWorldSeparatedStepInvariants(*world, sphereCount);

  state.counters["sphere_count"] = static_cast<double>(sphereCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["body_count"] = static_cast<double>(sphereCount);
  state.counters["contact_count"] = static_cast<double>(sphereCount);
  state.counters["max_height_error"] = check.maxHeightError;
  state.counters["max_vertical_speed"] = check.maxVerticalSpeed;
  state.counters["max_final_tangential_speed"] = check.maxFinalTangentialSpeed;
  state.counters["min_tangential_speed_drop"] = check.minTangentialSpeedDrop;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * sphereCount);
  state.SetLabel("BoxedLcpContact/WorldSeparatedStep");
}

static void BM_LcpWorldBoxStep_BoxedLcp(benchmark::State& state)
{
  const int boxCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldBoxStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldBoxStepBenchmarkWorld(boxCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->enterSimulationMode();
    world->step(stepCount);
    check = CheckWorldBoxStepInvariants(*world, boxCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldBoxStepBenchmarkWorld(boxCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->enterSimulationMode();
  world->step(stepCount);
  check = CheckWorldBoxStepInvariants(*world, boxCount);

  state.counters["box_count"] = static_cast<double>(boxCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["body_count"] = static_cast<double>(boxCount);
  state.counters["contact_count"] = static_cast<double>(4 * boxCount);
  state.counters["dense_box_contact"] = 1.0;
  state.counters["max_height_error"] = check.maxHeightError;
  state.counters["max_vertical_speed"] = check.maxVerticalSpeed;
  state.counters["max_final_tangential_speed"] = check.maxFinalTangentialSpeed;
  state.counters["min_tangential_speed_drop"] = check.minTangentialSpeedDrop;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * boxCount);
  state.SetLabel("BoxedLcpContact/WorldBoxStep");
}

static void BM_LcpWorldBilliardsStep_BoxedLcp(benchmark::State& state)
{
  const int pairCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldBilliardsStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldBilliardsStepBenchmarkWorld(pairCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->enterSimulationMode();
    world->step(stepCount);
    check = CheckWorldBilliardsStepInvariants(*world, pairCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldBilliardsStepBenchmarkWorld(pairCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->enterSimulationMode();
  world->step(stepCount);
  check = CheckWorldBilliardsStepInvariants(*world, pairCount);

  state.counters["pair_count"] = static_cast<double>(pairCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["body_count"] = static_cast<double>(2 * pairCount);
  state.counters["contact_count"] = static_cast<double>(check.contactCount);
  state.counters["max_momentum_error"] = check.maxMomentumError;
  state.counters["max_energy_error"] = check.maxEnergyError;
  state.counters["min_target_speed"] = check.minTargetSpeed;
  state.counters["max_target_speed"] = check.maxTargetSpeed;
  state.counters["target_speed_spread"] = check.targetSpeedSpread;
  state.counters["max_cue_speed"] = check.maxCueSpeed;
  state.counters["max_off_axis_speed"] = check.maxOffAxisSpeed;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * pairCount);
  state.SetLabel("BoxedLcpContact/WorldBilliardsStep");
}

static void BM_LcpWorldCardPileStep_BoxedLcp(benchmark::State& state)
{
  const int cardCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldCardPileStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldCardPileStepBenchmarkWorld(cardCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->enterSimulationMode();
    world->step(stepCount);
    check = CheckWorldCardPileStepInvariants(*world, cardCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldCardPileStepBenchmarkWorld(cardCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->enterSimulationMode();
  world->step(stepCount);
  check = CheckWorldCardPileStepInvariants(*world, cardCount);

  state.counters["card_count"] = static_cast<double>(cardCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["body_count"] = static_cast<double>(cardCount);
  state.counters["contact_count"] = static_cast<double>(check.contactCount);
  state.counters["thin_card_pile"] = 1.0;
  state.counters["max_card_spread"] = check.maxSpread;
  state.counters["max_height_loss"] = check.maxHeightLoss;
  state.counters["max_vertical_speed"] = check.maxVerticalSpeed;
  state.counters["max_angular_speed"] = check.maxAngularSpeed;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * cardCount);
  state.SetLabel("BoxedLcpContact/WorldCardPileStep");
}

static void BM_LcpWorldArticulatedGroundStep_BoxedLcp(benchmark::State& state)
{
  const int linkCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldArticulatedGroundStepCheck check;
  for (auto _ : state) {
    auto world
        = MakeWorldArticulatedGroundStepBenchmarkWorld(linkCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->step(stepCount);
    check = CheckWorldArticulatedGroundStepInvariants(*world, linkCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world
      = MakeWorldArticulatedGroundStepBenchmarkWorld(linkCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->step(stepCount);
  check = CheckWorldArticulatedGroundStepInvariants(*world, linkCount);

  state.counters["articulated_link_count"] = static_cast<double>(linkCount);
  state.counters["multibody_count"] = static_cast<double>(linkCount);
  state.counters["contact_count"] = static_cast<double>(linkCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["max_height_error"] = check.maxHeightError;
  state.counters["max_abs_joint_velocity"] = check.maxAbsJointVelocity;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * linkCount);
  state.SetLabel("BoxedLcpContact/WorldArticulatedGroundStep");
}

static void BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp(
    benchmark::State& state)
{
  const int pairCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldArticulatedRigidImpactStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldArticulatedRigidImpactStepBenchmarkWorld(
        pairCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->step(stepCount);
    check = CheckWorldArticulatedRigidImpactStepInvariants(*world, pairCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldArticulatedRigidImpactStepBenchmarkWorld(
      pairCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->step(stepCount);
  check = CheckWorldArticulatedRigidImpactStepInvariants(*world, pairCount);

  state.counters["articulated_link_count"] = static_cast<double>(pairCount);
  state.counters["dynamic_rigid_body_count"] = static_cast<double>(pairCount);
  state.counters["contact_count"] = static_cast<double>(pairCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["max_momentum_error"] = check.maxMomentumError;
  state.counters["min_target_velocity"] = check.minTargetVelocity;
  state.counters["max_striker_velocity"] = check.maxStrikerVelocity;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * pairCount);
  state.SetLabel("BoxedLcpContact/WorldArticulatedRigidImpactStep");
}

static void BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp(
    benchmark::State& state)
{
  const int pairCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldArticulatedLinkImpactStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldArticulatedLinkImpactStepBenchmarkWorld(
        pairCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->step(stepCount);
    check = CheckWorldArticulatedLinkImpactStepInvariants(*world, pairCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldArticulatedLinkImpactStepBenchmarkWorld(
      pairCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->step(stepCount);
  check = CheckWorldArticulatedLinkImpactStepInvariants(*world, pairCount);

  state.counters["articulated_pair_count"] = static_cast<double>(pairCount);
  state.counters["articulated_link_count"] = static_cast<double>(2 * pairCount);
  state.counters["articulated_dof_count"] = static_cast<double>(2 * pairCount);
  state.counters["multibody_count"] = static_cast<double>(2 * pairCount);
  state.counters["contact_count"] = static_cast<double>(pairCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["cross_multibody_link_contact"] = 1.0;
  state.counters["max_momentum_error"] = check.maxMomentumError;
  state.counters["min_target_velocity"] = check.minTargetVelocity;
  state.counters["max_striker_velocity"] = check.maxStrikerVelocity;
  state.counters["min_relative_velocity"] = check.minRelativeVelocity;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * pairCount);
  state.SetLabel("BoxedLcpContact/WorldArticulatedLinkImpactStep");
}

static void BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp(
    benchmark::State& state)
{
  const int chainCount = static_cast<int>(state.range(0));
  const int stepCount = static_cast<int>(state.range(1));
  std::string errorMessage;

  WorldCartesianArticulatedGroundStepCheck check;
  for (auto _ : state) {
    auto world = MakeWorldCartesianArticulatedGroundStepBenchmarkWorld(
        chainCount, errorMessage);
    if (world == nullptr) {
      state.SkipWithError(errorMessage.c_str());
      return;
    }
    world->step(stepCount);
    check = CheckWorldCartesianArticulatedGroundStepInvariants(
        *world, chainCount);
    benchmark::DoNotOptimize(check.ok);
  }

  auto world = MakeWorldCartesianArticulatedGroundStepBenchmarkWorld(
      chainCount, errorMessage);
  if (world == nullptr) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  world->step(stepCount);
  check
      = CheckWorldCartesianArticulatedGroundStepInvariants(*world, chainCount);

  state.counters["cartesian_chain_count"] = static_cast<double>(chainCount);
  state.counters["articulated_dof_count"] = static_cast<double>(3 * chainCount);
  state.counters["multibody_count"] = static_cast<double>(chainCount);
  state.counters["contact_count"] = static_cast<double>(chainCount);
  state.counters["step_count"] = static_cast<double>(stepCount);
  state.counters["serial_prismatic_chain"] = 1.0;
  state.counters["max_height_error"] = check.maxHeightError;
  state.counters["max_abs_joint_velocity"] = check.maxAbsJointVelocity;
  state.counters["max_planar_joint_speed"] = check.maxPlanarJointSpeed;
  state.counters["invariant_ok"] = check.ok ? 1.0 : 0.0;
  AddBackendBuildCounters(state);
  state.SetItemsProcessed(state.iterations() * stepCount * chainCount);
  state.SetLabel("BoxedLcpContact/WorldArticulatedCartesianGroundStep");
}
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RunManifestParallelBatchBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const int problemArg = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  ParallelBatchFixture fixture(solverEntry, family, problemArg, batchSize);
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(fixture.problems.front().b.size()),
      batchSize,
      MakeLabel(
          std::string(solverEntry.name),
          "BatchParallel/" + std::string(getProblemFamilyName(family))));
  AddParallelExecutionCounters(state, profile, fixture.graph);

  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
  if (fixture.storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, fixture.storage.shockPropagationParams);
  }
}

void RunGroupedBatchParallelBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    BenchmarkProblemFamily family)
{
  const int variantsPerProblemArg = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto batch
      = MakeGroupedBenchmarkBatch(family, variantsPerProblemArg, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }
  ParallelBatchFixture fixture(
      solverEntry, FlattenGroupedBenchmarkBatch(*batch));
  if (!fixture.valid) {
    state.SkipWithError(fixture.errorMessage.c_str());
    return;
  }

  compute::ParallelExecutor executor;
  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    executor.execute(fixture.graph);
    counters = fixture.collectCounters();
    benchmark::DoNotOptimize(counters.maxResidual);
  }

  const auto profile = executor.executeProfiled(fixture.graph);
  counters = fixture.collectCounters();
  AddGroupedBatchBenchmarkCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          std::string(solverEntry.name),
          "GroupedBatchParallel/" + std::string(getProblemFamilyName(family))));
  state.counters["grouped_batch_parallel"] = 1.0;
  AddParallelExecutionCounters(state, profile, fixture.graph);
}
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
void RunCudaJacobiBatchBenchmark(
    benchmark::State& state, BenchmarkProblemFamily family)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int problemArg = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  const auto problems
      = MakeBenchmarkProblemBatch(family, problemArg, batchSize);

  constexpr std::size_t iterations = 200;
  auto basePacket = MakeCudaBatchProblem(problems, iterations);
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
  counters = CheckCudaBatchResult(problems, packet.x, options, iterations);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel("JacobiCuda", std::string(getProblemFamilyName(family))));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
}

void RunCudaPgsBatchBenchmark(
    benchmark::State& state, BenchmarkProblemFamily family)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int problemArg = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  const auto problems
      = MakeBenchmarkProblemBatch(family, problemArg, batchSize);

  constexpr std::size_t iterations = 100;
  auto basePacket = MakeCudaBatchProblem(problems, iterations);
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
  counters = CheckCudaBatchResult(problems, packet.x, options, iterations);
  AddBatchBenchmarkCounters(
      state,
      counters,
      static_cast<int>(problems.front().b.size()),
      batchSize,
      MakeLabel("PgsCuda", std::string(getProblemFamilyName(family))));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
}

void RunCudaGroupedBatchBenchmark(
    benchmark::State& state, BenchmarkProblemFamily family, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerProblemArg = static_cast<int>(state.range(0));
  const std::size_t iterations = usePgs ? 100u : 200u;
  std::string errorMessage;
  auto batch = MakeGroupedCudaBenchmarkBatch(
      family, variantsPerProblemArg, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = batch->packets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = batch->packets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->grouped.problemGroups, packets, options, iterations);
  AddCudaGroupedBenchmarkCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "GroupedBatch/" + std::string(getProblemFamilyName(family))));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
}
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION                                         \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
void RunCudaWorldContactBatchBenchmark(benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int contactCount = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  std::string errorMessage;
  const auto batch = MakeHomogeneousWorldContactBenchmarkBatch(
      contactCount, batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const std::size_t iterations = usePgs ? 256u : 512u;
  auto basePacket = MakeCudaBatchProblem(batch->problems, iterations);
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
    } else {
      cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
    }
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
  } else {
    cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
  }
  counters
      = CheckCudaBatchResult(batch->problems, packet.x, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldContactBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["contact_count"] = contactCount;
  state.counters["problem_size"] = static_cast<double>(3 * contactCount);
}

void RunCudaWorldBoxContactBatchBenchmark(benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int boxCount = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  std::string errorMessage;
  const auto batch = MakeHomogeneousWorldBoxContactBenchmarkBatch(
      boxCount, batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const std::size_t iterations = usePgs ? 1024u : 8192u;
  auto basePacket = MakeCudaBatchProblem(batch->problems, iterations);
  if (!usePgs) {
    basePacket.relaxation = 0.25;
  }
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
    } else {
      cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
    }
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
  } else {
    cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
  }
  counters
      = CheckCudaBatchResult(batch->problems, packet.x, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldBoxContactBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_dense_box_contact_batch"] = 1.0;
  state.counters["dense_box_contact"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_relaxation"] = basePacket.relaxation;
  state.counters["box_count"] = static_cast<double>(boxCount);
  state.counters["contact_count"] = static_cast<double>(4 * boxCount);
  state.counters["problem_size"] = static_cast<double>(12 * boxCount);
}

void RunCudaWorldBoxContactGroupedBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerBoxCount = static_cast<int>(state.range(0));
  const std::size_t iterations = usePgs ? 1024u : 8192u;
  std::string errorMessage;
  const auto batch = MakeGroupedWorldBoxContactCudaBatch(
      variantsPerBoxCount, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));
  auto basePackets = batch->packets;
  if (!usePgs) {
    for (auto& packet : basePackets) {
      packet.relaxation = 0.25;
    }
  }

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = basePackets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = basePackets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldBoxContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_dense_box_contact_batch"] = 1.0;
  state.counters["dense_box_contact"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_relaxation"] = usePgs ? 1.0 : 0.25;
  state.counters["cuda_group_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["box_count_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["contact_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["problem_variants_per_shape"] = variantsPerBoxCount;
  state.counters["min_problem_size"]
      = static_cast<double>(batch->minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch->maxProblemSize);
}

void RunCudaWorldStackContactBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int sphereCount = static_cast<int>(state.range(0));
  const int batchSize = static_cast<int>(state.range(1));
  std::string errorMessage;
  const auto batch = MakeHomogeneousWorldStackContactBenchmarkBatch(
      sphereCount, batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  constexpr std::size_t iterations = 8192u;
  auto basePacket = MakeCudaBatchProblem(batch->problems, iterations);
  if (!usePgs) {
    basePacket.relaxation = 0.25;
  }
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
    } else {
      cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
    }
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
  } else {
    cuda_compute::solveBoxedLcpJacobiBatchCuda(packet);
  }
  counters
      = CheckCudaBatchResult(batch->problems, packet.x, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldStackContactBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_world_stack_contact_batch"] = 1.0;
  state.counters["cuda_coupled_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_relaxation"] = usePgs ? 1.0 : 0.25;
  state.counters["sphere_count"] = sphereCount;
  state.counters["contact_count"] = sphereCount;
  state.counters["problem_size"] = static_cast<double>(3 * sphereCount);
}

void RunCudaWorldContactGroupedBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerContactCount = static_cast<int>(state.range(0));
  const std::size_t iterations = usePgs ? 256u : 512u;
  std::string errorMessage;
  const auto batch = MakeGroupedWorldContactCudaBatch(
      variantsPerContactCount, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = batch->packets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = batch->packets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_group_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["contact_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["problem_variants_per_shape"] = variantsPerContactCount;
  state.counters["min_problem_size"]
      = static_cast<double>(batch->minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch->maxProblemSize);
}

void RunCudaWorldStackContactGroupedBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerSphereCount = static_cast<int>(state.range(0));
  constexpr std::size_t iterations = 8192u;
  std::string errorMessage;
  const auto batch = MakeGroupedWorldStackContactCudaBatch(
      variantsPerSphereCount, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  auto basePackets = batch->packets;
  if (!usePgs) {
    for (auto& packet : basePackets) {
      packet.relaxation = 0.25;
    }
  }
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = basePackets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = basePackets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "WorldStackContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_world_stack_contact_batch"] = 1.0;
  state.counters["cuda_coupled_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_relaxation"] = usePgs ? 1.0 : 0.25;
  state.counters["cuda_group_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["contact_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["problem_variants_per_shape"] = variantsPerSphereCount;
  state.counters["min_problem_size"]
      = static_cast<double>(batch->minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch->maxProblemSize);
}

void RunCudaArticulatedUnifiedContactGroupedBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerContactCount = static_cast<int>(state.range(0));
  const std::size_t iterations = usePgs ? 512u : 1024u;
  std::string errorMessage;
  const auto batch = MakeGroupedArticulatedUnifiedContactCudaBatch(
      variantsPerContactCount, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = batch->packets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = batch->packets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "ArticulatedUnifiedContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_articulated_contact_batch"] = 1.0;
  state.counters["cuda_articulated_unified_contact_batch"] = 1.0;
  state.counters["articulated_cross_link_contact"] = 1.0;
  state.counters["articulated_contact_case_count"] = 3.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_group_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["contact_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["problem_variants_per_shape"] = variantsPerContactCount;
  state.counters["min_problem_size"]
      = static_cast<double>(batch->minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch->maxProblemSize);
}

void RunCudaMixedContactGroupedBatchBenchmark(
    benchmark::State& state, bool usePgs)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerScenario = static_cast<int>(state.range(0));
  const std::size_t iterations = usePgs ? 512u : 1024u;
  std::string errorMessage;
  const auto batch = MakeGroupedMixedContactCudaBatch(
      variantsPerScenario, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = batch->packets;
    if (usePgs) {
      cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    } else {
      cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
    }
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = batch->packets;
  if (usePgs) {
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  } else {
    cuda_compute::solveBoxedLcpJacobiGroupedBatchCuda(packets);
  }
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel(
          usePgs ? "PgsCuda" : "JacobiCuda",
          "MixedContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_mixed_contact_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_world_stack_contact_batch"] = 1.0;
  state.counters["cuda_coupled_contact_batch"] = 1.0;
  state.counters["cuda_articulated_contact_batch"] = 1.0;
  state.counters["cuda_articulated_unified_contact_batch"] = 1.0;
  state.counters["articulated_cross_link_contact"] = 1.0;
  state.counters["articulated_contact_case_count"] = 3.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["cuda_group_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["contact_fixture_family_count"] = 3.0;
  state.counters["contact_shape_count"]
      = static_cast<double>(batch->packets.size());
  state.counters["problem_variants_per_shape"] = variantsPerScenario;
  state.counters["min_problem_size"]
      = static_cast<double>(batch->minProblemSize);
  state.counters["max_problem_size"]
      = static_cast<double>(batch->maxProblemSize);
}
#endif

std::vector<int> GetBenchmarkProblemArgCandidates(
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  if (family == BenchmarkProblemFamily::Standard && solver.name == "Direct") {
    return {2, 3};
  }

  switch (family) {
    case BenchmarkProblemFamily::Standard:
      return {12, 24, 48, 96};
    case BenchmarkProblemFamily::Boxed:
      return {12, 24, 48};
    case BenchmarkProblemFamily::FrictionIndex:
      return {4, 16, 64};
  }

  return {};
}

std::vector<int> GetConcreteBenchmarkArgs(
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  std::vector<int> args;
  for (const int problemArg :
       GetBenchmarkProblemArgCandidates(solver, family)) {
    if (SolverSupportsConcreteProblem(
            solver, MakeBenchmarkProblem(family, problemArg))) {
      args.push_back(problemArg);
    }
  }

  return args;
}

void AddBenchmarkArgs(
    benchmark::Benchmark* registeredBenchmark, const std::vector<int>& args)
{
  for (const int problemArg : args) {
    registeredBenchmark->Arg(problemArg);
  }
}

struct BatchBenchmarkArg
{
  int problemArg{0};
  int batchSize{0};
};

std::vector<BatchBenchmarkArg> GetBatchBenchmarkArgCandidates(
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  constexpr int batchSize = 4;
  const bool addCudaComparableSizes
      = solver.name == "Jacobi" || solver.name == "Pgs";
  if (family == BenchmarkProblemFamily::Standard && solver.name == "Direct") {
    return {{3, batchSize}};
  }

  switch (family) {
    case BenchmarkProblemFamily::Standard: {
      std::vector<BatchBenchmarkArg> args{{24, batchSize}};
      if (addCudaComparableSizes) {
        args.push_back({48, batchSize});
        args.push_back({96, batchSize});
        args.push_back({128, batchSize});
        args.push_back({192, batchSize});
        args.push_back({256, batchSize});
      }
      return args;
    }
    case BenchmarkProblemFamily::Boxed: {
      std::vector<BatchBenchmarkArg> args{{24, batchSize}};
      if (addCudaComparableSizes) {
        args.push_back({48, batchSize});
        args.push_back({96, batchSize});
        args.push_back({128, batchSize});
        args.push_back({192, batchSize});
        args.push_back({256, batchSize});
      }
      return args;
    }
    case BenchmarkProblemFamily::FrictionIndex: {
      std::vector<BatchBenchmarkArg> args{{8, batchSize}};
      if (addCudaComparableSizes) {
        args.push_back({16, batchSize});
        args.push_back({32, batchSize});
        args.push_back({48, batchSize});
        args.push_back({64, batchSize});
        args.push_back({96, batchSize});
      }
      return args;
    }
  }

  return {};
}

std::vector<BatchBenchmarkArg> GetConcreteBatchBenchmarkArgs(
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  std::vector<BatchBenchmarkArg> args;
  for (const auto candidate : GetBatchBenchmarkArgCandidates(solver, family)) {
    if (SolverSupportsConcreteProblemBatch(
            solver,
            MakeBenchmarkProblemBatch(
                family, candidate.problemArg, candidate.batchSize))) {
      args.push_back(candidate);
    }
  }

  return args;
}

void AddBatchBenchmarkArgs(
    benchmark::Benchmark* registeredBenchmark,
    const std::vector<BatchBenchmarkArg>& args)
{
  for (const auto arg : args) {
    registeredBenchmark->Args({arg.problemArg, arg.batchSize});
  }
}

std::vector<int> GetConcreteGroupedBatchBenchmarkArgs(
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  std::vector<int> args;
  for (const int variantsPerProblemArg : std::array<int, 2>{2, 3}) {
    std::string errorMessage;
    const auto batch = MakeGroupedBenchmarkBatch(
        family, variantsPerProblemArg, errorMessage);
    if (!batch.has_value()) {
      continue;
    }

    if (SolverSupportsConcreteProblemBatch(
            solver, FlattenGroupedBenchmarkBatch(*batch))) {
      args.push_back(variantsPerProblemArg);
    }
  }

  return args;
}

std::string MakeBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpCompare/" << getProblemFamilyName(family) << "/" << solver.name;
  return out.str();
}

std::string MakeActiveSetTransitionBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpActiveSetTransition/" << getProblemFamilyName(family) << "/"
      << solver.name;
  return out.str();
}

std::string MakeActiveFrictionIndexContactBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpActiveFrictionIndexContact/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeNewtonWarmStartBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const NewtonWarmStartMode mode)
{
  std::ostringstream out;
  out << "BM_LcpNewtonWarmStart/StandardActiveSet/" << solver.name << "/"
      << getNewtonWarmStartModeName(mode);
  return out.str();
}

std::string MakePgsRelaxationSweepBenchmarkName(
    const RelaxationSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpPgsRelaxationSweep/" << getProblemFamilyName(testCase.family)
      << "/" << getRelaxationSweepKindName(testCase.relaxationKind) << "/"
      << testCase.relaxationLabel;
  return out.str();
}

std::string MakeSymmetricPsorRelaxationSweepBenchmarkName(
    const RelaxationSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpSymmetricPsorRelaxationSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << getRelaxationSweepKindName(testCase.relaxationKind) << "/"
      << testCase.relaxationLabel;
  return out.str();
}

std::string MakeRedBlackGaussSeidelRelaxationSweepBenchmarkName(
    const RelaxationSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpRedBlackGaussSeidelRelaxationSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << getRelaxationSweepKindName(testCase.relaxationKind) << "/"
      << testCase.relaxationLabel;
  return out.str();
}

std::string MakeBoxedSsnLineSearchSweepBenchmarkName(
    const BoxedSsnLineSearchSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << testCase.lineSearchLabel;
  return out.str();
}

std::string MakePivotingScaleSweepBenchmarkName(
    const PivotingScaleSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpPivotingScaleSweep/" << getProblemFamilyName(testCase.family)
      << "/" << testCase.solverName << "/" << testCase.problemLabel;
  return out.str();
}

std::string MakeBlockPartitionSweepBenchmarkName(
    const BlockPartitionSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpBlockPartitionSweep/" << getProblemFamilyName(testCase.family)
      << "/" << testCase.solverName << "/" << testCase.profileLabel;
  return out.str();
}

std::string MakeApgdRestartSweepBenchmarkName(
    const ApgdRestartSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpApgdRestartSweep/" << getProblemFamilyName(testCase.family)
      << "/" << testCase.restartPolicyLabel;
  return out.str();
}

std::string MakeTgsIterationBudgetSweepBenchmarkName(
    const TgsIterationBudgetSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpTgsIterationBudgetSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << testCase.iterationBudgetLabel;
  return out.str();
}

std::string MakeNncgPgsIterationsSweepBenchmarkName(
    const NncgPgsIterationsSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpNncgPgsIterationsSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << testCase.pgsIterationsLabel;
  return out.str();
}

std::string MakeSubspacePgsIterationsSweepBenchmarkName(
    const SubspacePgsIterationsSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpSubspaceMinimizationPgsIterationsSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << testCase.pgsIterationsLabel;
  return out.str();
}

std::string MakeShockPropagationLayerSweepBenchmarkName(
    const ShockPropagationLayerSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpShockPropagationLayerSweep/"
      << getProblemFamilyName(testCase.family) << "/" << testCase.profileLabel;
  return out.str();
}

std::string MakeMprgpSpdCheckSweepBenchmarkName(
    const MprgpSpdCheckSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpMprgpSpdCheckSweep/" << testCase.kindLabel << "/"
      << testCase.problemSize << "/" << testCase.pdCheckLabel;
  return out.str();
}

std::string MakeInteriorPointPathSweepBenchmarkName(
    const InteriorPointPathSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpInteriorPointPathSweep/" << testCase.kindLabel << "/"
      << testCase.problemSize << "/" << testCase.sigmaLabel << "/"
      << testCase.stepScaleLabel;
  return out.str();
}

std::string MakeAdmmRhoSweepBenchmarkName(const AdmmRhoSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpAdmmRhoSweep/" << getProblemFamilyName(testCase.family) << "/"
      << testCase.policyLabel << "/" << testCase.rhoLabel;
  return out.str();
}

std::string MakeSapRegularizationSweepBenchmarkName(
    const SapRegularizationSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpSapRegularizationSweep/"
      << getProblemFamilyName(testCase.family) << "/"
      << testCase.regularizationLabel;
  return out.str();
}

std::string MakeNewtonWarmStartBatchSerialBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const NewtonWarmStartMode mode)
{
  std::ostringstream out;
  out << "BM_LcpNewtonWarmStartBatchSerial/StandardActiveSet/" << solver.name
      << "/" << getNewtonWarmStartModeName(mode);
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeNewtonWarmStartBatchParallelBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const NewtonWarmStartMode mode)
{
  std::ostringstream out;
  out << "BM_LcpNewtonWarmStartBatchParallel/StandardActiveSet/" << solver.name
      << "/" << getNewtonWarmStartModeName(mode);
  return out.str();
}
#endif

std::string MakeLargerActiveSetTransitionBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpLargerActiveSetTransition/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeStressActiveSetTransitionBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpStressActiveSetTransition/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeExtremeActiveSetTransitionBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpExtremeActiveSetTransition/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeProductionActiveSetTransitionBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpProductionActiveSetTransition/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeProductionActiveSetTransitionBatchSerialBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpProductionActiveSetTransitionBatchSerial/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeProductionActiveSetTransitionBatchParallelBenchmarkName(
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpProductionActiveSetTransitionBatchParallel/"
      << getLargerActiveSetTransitionCaseName(testCase) << "/" << solver.name;
  return out.str();
}
#endif

std::string MakeBatchBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpBatchSerial/" << getProblemFamilyName(family) << "/"
      << solver.name;
  return out.str();
}

std::string MakeGroupedBatchSerialBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpGroupedBatchSerial/" << getProblemFamilyName(family) << "/"
      << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeParallelBatchBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpBatchParallel/" << getProblemFamilyName(family) << "/"
      << solver.name;
  return out.str();
}

std::string MakeGroupedBatchParallelBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpGroupedBatchParallel/" << getProblemFamilyName(family) << "/"
      << solver.name;
  return out.str();
}

std::string MakeWorldContactBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContact/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldBoxContactBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldBoxContact/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldStackContactBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldStackContact/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeArticulatedUnifiedContactBenchmarkName(
    ArticulatedContactBenchmarkCase benchmarkCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpArticulatedUnifiedContact/FrictionIndex/"
      << getArticulatedContactBenchmarkCaseName(benchmarkCase) << "/"
      << solver.name;
  return out.str();
}

std::string MakeWorldContactBatchSerialBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactBatchSerial/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldContactBatchParallelBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactBatchParallel/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldContactStressBatchSerialBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactStressBatchSerial/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldContactStressBatchParallelBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactStressBatchParallel/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldContactPipeline32BatchSerialBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactPipeline32BatchSerial/FrictionIndex/"
      << solver.name;
  return out.str();
}

std::string MakeWorldContactPipeline32BatchParallelBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldContactPipeline32BatchParallel/FrictionIndex/"
      << solver.name;
  return out.str();
}

std::string MakeWorldBoxContactBatchSerialBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldBoxContactBatchSerial/FrictionIndex/" << solver.name;
  return out.str();
}

std::string MakeWorldBoxContactBatchParallelBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpWorldBoxContactBatchParallel/FrictionIndex/" << solver.name;
  return out.str();
}
#endif

std::string MakeStaggeringContactPipelineSweepBenchmarkName(
    const StaggeringContactPipelineSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpStaggeringContactPipelineSweep/" << testCase.caseLabel;
  return out.str();
}

std::string MakeContactSolverComparisonSweepBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const StaggeringContactPipelineSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpContactSolverComparisonSweep/" << solver.name << "/"
      << testCase.caseLabel;
  return out.str();
}

std::string MakeContactNormalStandardSweepBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const StaggeringContactPipelineSweepCase testCase)
{
  std::ostringstream out;
  out << "BM_LcpContactNormalStandardSweep/" << solver.name << "/"
      << testCase.caseLabel;
  return out.str();
}

std::string MakeMildIllConditionedBenchmarkName(
    const MildIllConditionedBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpMildIllConditioned/" << getMildIllConditionedCaseName(testCase)
      << "/" << solver.name;
  return out.str();
}

std::string MakeMildIllConditionedBatchSerialBenchmarkName(
    const MildIllConditionedBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpMildIllConditionedBatchSerial/"
      << getMildIllConditionedCaseName(testCase) << "/" << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeMildIllConditionedBatchParallelBenchmarkName(
    const MildIllConditionedBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpMildIllConditionedBatchParallel/"
      << getMildIllConditionedCaseName(testCase) << "/" << solver.name;
  return out.str();
}
#endif

bool SolverShouldRunMildIllConditionedBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const MildIllConditionedBenchmarkCase testCase,
    const LcpProblem& problem)
{
  constexpr std::array<std::string_view, 14> kScopedSolvers{{
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

  if (testCase == MildIllConditionedBenchmarkCase::Standard32) {
    constexpr std::array<std::string_view, 2> kExactStandardSolvers{{
        "Dantzig",
        "Baraff",
    }};
    return (SolverNameIn(solver, kExactStandardSolvers)
            || SolverNameIn(solver, kScopedSolvers))
           && SolverSupportsConcreteProblem(solver, problem);
  }

  if (isMildIllConditionedCoupledFrictionIndexCase(testCase)
      && getMildIllConditionedCouplingScale(testCase) > 8.0) {
    return (SolverNameIn(solver, kScopedSolvers)
            || solver.name == "BoxedSemiSmoothNewton")
           && SolverSupportsConcreteProblem(solver, problem);
  }

  if (isMildIllConditionedCoupledFrictionIndexCase(testCase)
      && solver.name == "BoxedSemiSmoothNewton") {
    return SolverSupportsConcreteProblem(solver, problem);
  }

  return SolverNameIn(solver, kScopedSolvers)
         && SolverSupportsConcreteProblem(solver, problem);
}

bool SolverShouldRunMildIllConditionedBatchBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const MildIllConditionedBenchmarkCase testCase,
    const std::vector<LcpProblem>& problems)
{
  if (problems.empty()) {
    return false;
  }

  return SolverShouldRunMildIllConditionedBenchmark(
             solver, testCase, problems.front())
         && SolverSupportsConcreteProblemBatch(solver, problems);
}

std::string MakeNearSingularBenchmarkName(
    const NearSingularBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpNearSingular/" << getNearSingularCaseName(testCase) << "/"
      << solver.name;
  return out.str();
}

std::string MakeNearSingularBatchSerialBenchmarkName(
    const NearSingularBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpNearSingularBatchSerial/" << getNearSingularCaseName(testCase)
      << "/" << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeNearSingularBatchParallelBenchmarkName(
    const NearSingularBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpNearSingularBatchParallel/" << getNearSingularCaseName(testCase)
      << "/" << solver.name;
  return out.str();
}
#endif

bool SolverShouldRunNearSingularBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const NearSingularBenchmarkCase testCase,
    const LcpProblem& problem)
{
  if (testCase == NearSingularBenchmarkCase::Standard8) {
    constexpr std::array<std::string_view, 2> kStandardSolvers{{
        "Dantzig",
        "Baraff",
    }};
    return SolverNameIn(solver, kStandardSolvers)
           && SolverSupportsConcreteProblem(solver, problem);
  }

  if (testCase == NearSingularBenchmarkCase::Boxed8) {
    constexpr std::array<std::string_view, 3> kBoxedSolvers{{
        "Dantzig",
        "ShockPropagation",
        "BoxedSemiSmoothNewton",
    }};
    return SolverNameIn(solver, kBoxedSolvers)
           && SolverSupportsConcreteProblem(solver, problem);
  }

  constexpr std::array<std::string_view, 2> kFrictionIndexSolvers{{
      "Dantzig",
      "ShockPropagation",
  }};
  return SolverNameIn(solver, kFrictionIndexSolvers)
         && SolverSupportsConcreteProblem(solver, problem);
}

bool SolverShouldRunNearSingularBatchBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const NearSingularBenchmarkCase testCase,
    const std::vector<LcpProblem>& problems)
{
  if (problems.empty()) {
    return false;
  }

  return SolverShouldRunNearSingularBenchmark(
             solver, testCase, problems.front())
         && SolverSupportsConcreteProblemBatch(solver, problems);
}

bool SolverShouldRunLargerActiveSetTransitionBenchmark(
    const dart::test::LcpSolverManifestEntry& solver, const LcpProblem& problem)
{
  constexpr std::array<std::string_view, 17> kScalableSolvers{{
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

  return SolverNameIn(solver, kScalableSolvers)
         && SolverSupportsConcreteProblem(solver, problem);
}

bool SolverShouldRunProductionActiveSetTransitionBatchBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const std::vector<LcpProblem>& problems)
{
  if (problems.empty()) {
    return false;
  }

  return SolverShouldRunLargerActiveSetTransitionBenchmark(
             solver, problems.front())
         && SolverSupportsConcreteProblemBatch(solver, problems);
}

std::string MakeSingularDegenerateBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpSingularDegenerate/" << getSingularDegenerateCaseName(testCase)
      << "/" << solver.name;
  return out.str();
}

std::string MakeSingularDegenerateFrictionIndexBatchSerialBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpSingularDegenerateFrictionIndexBatchSerial/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeSingularDegenerateFrictionIndexBatchParallelBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpSingularDegenerateFrictionIndexBatchParallel/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}
#endif

std::string MakeSingularDegenerateStandardBoxedBatchSerialBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpSingularDegenerateStandardBoxedBatchSerial/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
std::string MakeSingularDegenerateStandardBoxedBatchParallelBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpSingularDegenerateStandardBoxedBatchParallel/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}
#endif

bool SolverShouldRunSingularDegenerateBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const SingularDegenerateBenchmarkCase testCase,
    const LcpProblem& problem)
{
  if (testCase == SingularDegenerateBenchmarkCase::Standard16
      || testCase == SingularDegenerateBenchmarkCase::Standard32
      || testCase == SingularDegenerateBenchmarkCase::Standard64
      || testCase == SingularDegenerateBenchmarkCase::Standard128) {
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
    return SolverNameIn(solver, kStandardSolvers)
           && SolverSupportsConcreteProblem(solver, problem);
  }

  constexpr std::array<std::string_view, 3> kBoxedAndFindexSolvers{{
      "Admm",
      "Sap",
      "BoxedSemiSmoothNewton",
  }};
  return SolverNameIn(solver, kBoxedAndFindexSolvers)
         && SolverSupportsConcreteProblem(solver, problem);
}

bool SolverShouldRunSingularDegenerateBatchBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const SingularDegenerateBenchmarkCase testCase,
    const std::vector<LcpProblem>& problems)
{
  if (problems.empty()) {
    return false;
  }

  return SolverShouldRunSingularDegenerateBenchmark(
             solver, testCase, problems.front())
         && SolverSupportsConcreteProblemBatch(solver, problems);
}

std::string MakeLargerSingularDegenerateBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpLargerSingularDegenerate/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeStressSingularDegenerateBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpStressSingularDegenerate/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}

std::string MakeExtremeSingularDegenerateBenchmarkName(
    const SingularDegenerateBenchmarkCase testCase,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpExtremeSingularDegenerate/"
      << getSingularDegenerateCaseName(testCase) << "/" << solver.name;
  return out.str();
}

void RegisterManifestBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      const auto args = GetConcreteBenchmarkArgs(solver, family);
      if (args.empty()) {
        continue;
      }

      const auto name = MakeBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestBenchmark(state, solver, family);
          });
      AddBenchmarkArgs(registeredBenchmark, args);
    }
  }
}

void RegisterActiveSetTransitionBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    const auto problem = MakeActiveSetTransitionBenchmarkProblem(family);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverSupportsConcreteProblem(solver, problem)) {
        continue;
      }

      const auto name = MakeActiveSetTransitionBenchmarkName(family, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunActiveSetTransitionBenchmark(state, solver, family);
          });
    }
  }
}

void RegisterActiveFrictionIndexContactBenchmarks()
{
  const auto problem = MakeActiveFrictionIndexContactProblem();
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!SolverSupportsConcreteProblem(solver, problem)) {
      continue;
    }

    const auto name = MakeActiveFrictionIndexContactBenchmarkName(solver);
    benchmark::RegisterBenchmark(
        name.c_str(), [solver](benchmark::State& state) {
          RunActiveFrictionIndexContactBenchmark(state, solver);
        });
  }
}

void RegisterPgsRelaxationSweepBenchmarks()
{
  for (const auto testCase : kRelaxationSweepCases) {
    const auto name = MakePgsRelaxationSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunPgsRelaxationSweepBenchmark(state, testCase);
        });
  }
}

void RegisterSymmetricPsorRelaxationSweepBenchmarks()
{
  for (const auto testCase : kRelaxationSweepCases) {
    const auto name = MakeSymmetricPsorRelaxationSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunSymmetricPsorRelaxationSweepBenchmark(state, testCase);
        });
  }
}

void RegisterRedBlackGaussSeidelRelaxationSweepBenchmarks()
{
  for (const auto testCase : kRelaxationSweepCases) {
    const auto name
        = MakeRedBlackGaussSeidelRelaxationSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunRedBlackGaussSeidelRelaxationSweepBenchmark(state, testCase);
        });
  }
}

void RegisterBoxedSsnLineSearchSweepBenchmarks()
{
  for (const auto testCase : kBoxedSsnLineSearchSweepCases) {
    const auto name = MakeBoxedSsnLineSearchSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunBoxedSsnLineSearchSweepBenchmark(state, testCase);
        });
  }
}

void RegisterPivotingScaleSweepBenchmarks()
{
  for (const auto testCase : kPivotingScaleSweepCases) {
    const auto* solverEntry = FindSolverManifestEntry(testCase.solverName);
    if (solverEntry == nullptr) {
      continue;
    }

    const auto problem
        = MakeBenchmarkProblem(testCase.family, testCase.problemArg);
    if (!SolverSupportsConcreteProblem(*solverEntry, problem)) {
      continue;
    }

    const auto name = MakePivotingScaleSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunPivotingScaleSweepBenchmark(state, testCase);
        });
  }
}

void RegisterBlockPartitionSweepBenchmarks()
{
  for (const auto testCase : kBlockPartitionSweepCases) {
    const auto name = MakeBlockPartitionSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunBlockPartitionSweepBenchmark(state, testCase);
        });
  }
}

void RegisterApgdRestartSweepBenchmarks()
{
  for (const auto testCase : kApgdRestartSweepCases) {
    const auto name = MakeApgdRestartSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunApgdRestartSweepBenchmark(state, testCase);
        });
  }
}

void RegisterTgsIterationBudgetSweepBenchmarks()
{
  for (const auto testCase : kTgsIterationBudgetSweepCases) {
    const auto name = MakeTgsIterationBudgetSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunTgsIterationBudgetSweepBenchmark(state, testCase);
        });
  }
}

void RegisterNncgPgsIterationsSweepBenchmarks()
{
  for (const auto testCase : kNncgPgsIterationsSweepCases) {
    const auto name = MakeNncgPgsIterationsSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunNncgPgsIterationsSweepBenchmark(state, testCase);
        });
  }
}

void RegisterSubspacePgsIterationsSweepBenchmarks()
{
  for (const auto testCase : kSubspacePgsIterationsSweepCases) {
    const auto name = MakeSubspacePgsIterationsSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunSubspacePgsIterationsSweepBenchmark(state, testCase);
        });
  }
}

void RegisterShockPropagationLayerSweepBenchmarks()
{
  for (const auto testCase : kShockPropagationLayerSweepCases) {
    const auto name = MakeShockPropagationLayerSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunShockPropagationLayerSweepBenchmark(state, testCase);
        });
  }
}

void RegisterMprgpSpdCheckSweepBenchmarks()
{
  for (const auto testCase : kMprgpSpdCheckSweepCases) {
    const auto name = MakeMprgpSpdCheckSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunMprgpSpdCheckSweepBenchmark(state, testCase);
        });
  }
}

void RegisterInteriorPointPathSweepBenchmarks()
{
  for (const auto& testCase : kInteriorPointPathSweepCases) {
    const auto benchmarkCase = testCase;
    const auto name = MakeInteriorPointPathSweepBenchmarkName(benchmarkCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [benchmarkCase](benchmark::State& state) {
          RunInteriorPointPathSweepBenchmark(state, benchmarkCase);
        });
  }
}

void RegisterAdmmRhoSweepBenchmarks()
{
  for (const auto testCase : kAdmmRhoSweepCases) {
    const auto name = MakeAdmmRhoSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunAdmmRhoSweepBenchmark(state, testCase);
        });
  }
}

void RegisterSapRegularizationSweepBenchmarks()
{
  for (const auto testCase : kSapRegularizationSweepCases) {
    const auto name = MakeSapRegularizationSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunSapRegularizationSweepBenchmark(state, testCase);
        });
  }
}

void RegisterNewtonWarmStartBenchmarks()
{
  constexpr std::array<std::string_view, 3> kStandardNewtonSolvers{{
      "MinimumMapNewton",
      "FischerBurmeisterNewton",
      "PenalizedFischerBurmeisterNewton",
  }};
  constexpr std::array<NewtonWarmStartMode, 4> kModes{{
      NewtonWarmStartMode::None,
      NewtonWarmStartMode::Pgs,
      NewtonWarmStartMode::GradientDescent,
      NewtonWarmStartMode::PgsThenGradient,
  }};

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!SolverNameIn(solver, kStandardNewtonSolvers)) {
      continue;
    }

    const auto problemSizes = GetConcreteNewtonWarmStartProblemSizes(solver);
    const auto batchArgs = GetConcreteNewtonWarmStartBatchArgs(solver);
    if (problemSizes.empty() && batchArgs.empty()) {
      continue;
    }

    for (const auto mode : kModes) {
      if (!problemSizes.empty()) {
        const auto name = MakeNewtonWarmStartBenchmarkName(solver, mode);
        auto* registeredBenchmark = benchmark::RegisterBenchmark(
            name.c_str(), [solver, mode](benchmark::State& state) {
              RunNewtonWarmStartBenchmark(state, solver, mode);
            });
        AddBenchmarkArgs(registeredBenchmark, problemSizes);
      }

      if (!batchArgs.empty()) {
        const auto serialBatchName
            = MakeNewtonWarmStartBatchSerialBenchmarkName(solver, mode);
        auto* serialBatchBenchmark = benchmark::RegisterBenchmark(
            serialBatchName.c_str(), [solver, mode](benchmark::State& state) {
              RunNewtonWarmStartBatchSerialBenchmark(state, solver, mode);
            });
        AddNewtonWarmStartBatchArgs(serialBatchBenchmark, batchArgs);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
        const auto parallelBatchName
            = MakeNewtonWarmStartBatchParallelBenchmarkName(solver, mode);
        auto* parallelBatchBenchmark = benchmark::RegisterBenchmark(
            parallelBatchName.c_str(), [solver, mode](benchmark::State& state) {
              RunNewtonWarmStartBatchParallelBenchmark(state, solver, mode);
            });
        AddNewtonWarmStartBatchArgs(parallelBatchBenchmark, batchArgs);
#endif
      }
    }
  }
}

void RegisterLargerActiveSetTransitionBenchmarks()
{
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 3> cases{
      LargerActiveSetTransitionBenchmarkCase::Standard32,
      LargerActiveSetTransitionBenchmarkCase::Boxed32,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8};

  for (const auto testCase : cases) {
    const auto problem
        = MakeLargerActiveSetTransitionBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(solver, problem)) {
        continue;
      }

      const auto name
          = MakeLargerActiveSetTransitionBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunLargerActiveSetTransitionBenchmark(
                state,
                solver,
                testCase,
                "LargerActiveSetTransition",
                "larger_active_set_transition");
          });
    }
  }
}

void RegisterStressActiveSetTransitionBenchmarks()
{
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 3> cases{
      LargerActiveSetTransitionBenchmarkCase::Standard64,
      LargerActiveSetTransitionBenchmarkCase::Boxed64,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12};

  for (const auto testCase : cases) {
    const auto problem
        = MakeLargerActiveSetTransitionBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(solver, problem)) {
        continue;
      }

      const auto name
          = MakeStressActiveSetTransitionBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunLargerActiveSetTransitionBenchmark(
                state,
                solver,
                testCase,
                "StressActiveSetTransition",
                "stress_active_set_transition");
          });
    }
  }
}

void RegisterExtremeActiveSetTransitionBenchmarks()
{
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 3> cases{
      LargerActiveSetTransitionBenchmarkCase::Standard128,
      LargerActiveSetTransitionBenchmarkCase::Boxed128,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16};

  for (const auto testCase : cases) {
    const auto problem
        = MakeLargerActiveSetTransitionBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(solver, problem)) {
        continue;
      }

      const auto name
          = MakeExtremeActiveSetTransitionBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunLargerActiveSetTransitionBenchmark(
                state,
                solver,
                testCase,
                "ExtremeActiveSetTransition",
                "extreme_active_set_transition");
          });
    }
  }
}

void RegisterProductionActiveSetTransitionBenchmarks()
{
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 8> cases{
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256};

  for (const auto testCase : cases) {
    const auto problem
        = MakeLargerActiveSetTransitionBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(solver, problem)) {
        continue;
      }

      const auto name
          = MakeProductionActiveSetTransitionBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunLargerActiveSetTransitionBenchmark(
                state,
                solver,
                testCase,
                "ProductionActiveSetTransition",
                "production_active_set_transition");
          });
    }
  }
}

void RegisterProductionActiveSetTransitionBatchBenchmarks()
{
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 17> cases{
      LargerActiveSetTransitionBenchmarkCase::Standard32,
      LargerActiveSetTransitionBenchmarkCase::Boxed32,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8,
      LargerActiveSetTransitionBenchmarkCase::Standard64,
      LargerActiveSetTransitionBenchmarkCase::Boxed64,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12,
      LargerActiveSetTransitionBenchmarkCase::Standard128,
      LargerActiveSetTransitionBenchmarkCase::Boxed128,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex48,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex64,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex96,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex128,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex192,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex256};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    const auto problems
        = MakeProductionActiveSetTransitionBatchProblems(testCase, batchSize);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunProductionActiveSetTransitionBatchBenchmark(
              solver, problems)) {
        continue;
      }

      const auto serialName
          = MakeProductionActiveSetTransitionBatchSerialBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunProductionActiveSetTransitionBatchSerialBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
      const auto parallelName
          = MakeProductionActiveSetTransitionBatchParallelBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunProductionActiveSetTransitionBatchParallelBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);
#endif
    }
  }
}

void RegisterMildIllConditionedBenchmarks()
{
  constexpr std::array<MildIllConditionedBenchmarkCase, 42> cases{
      MildIllConditionedBenchmarkCase::Standard32,
      MildIllConditionedBenchmarkCase::Boxed16,
      MildIllConditionedBenchmarkCase::FrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256};

  for (const auto testCase : cases) {
    const auto problem = MakeMildIllConditionedBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunMildIllConditionedBenchmark(
              solver, testCase, problem)) {
        continue;
      }

      const auto name = MakeMildIllConditionedBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunMildIllConditionedBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterMildIllConditionedBatchBenchmarks()
{
  constexpr std::array<MildIllConditionedBenchmarkCase, 42> cases{
      MildIllConditionedBenchmarkCase::Standard32,
      MildIllConditionedBenchmarkCase::Boxed16,
      MildIllConditionedBenchmarkCase::FrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex32,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex48,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex64,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex96,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex128,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex192,
      MildIllConditionedBenchmarkCase::ExtremeCoupledFrictionIndex256};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    const auto problems
        = MakeMildIllConditionedBatchProblems(testCase, batchSize);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunMildIllConditionedBatchBenchmark(
              solver, testCase, problems)) {
        continue;
      }

      const auto serialName
          = MakeMildIllConditionedBatchSerialBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunMildIllConditionedBatchSerialBenchmark(state, solver, testCase);
          })
          ->Arg(batchSize);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
      const auto parallelName
          = MakeMildIllConditionedBatchParallelBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunMildIllConditionedBatchParallelBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);
#endif
    }
  }
}

void RegisterNearSingularBenchmarks()
{
  constexpr std::array<NearSingularBenchmarkCase, 15> cases{
      NearSingularBenchmarkCase::Standard8,
      NearSingularBenchmarkCase::Boxed8,
      NearSingularBenchmarkCase::CoupledFrictionIndex3,
      NearSingularBenchmarkCase::CoupledFrictionIndex6,
      NearSingularBenchmarkCase::CoupledFrictionIndex9,
      NearSingularBenchmarkCase::CoupledFrictionIndex12,
      NearSingularBenchmarkCase::CoupledFrictionIndex16,
      NearSingularBenchmarkCase::CoupledFrictionIndex24,
      NearSingularBenchmarkCase::CoupledFrictionIndex32,
      NearSingularBenchmarkCase::CoupledFrictionIndex48,
      NearSingularBenchmarkCase::CoupledFrictionIndex64,
      NearSingularBenchmarkCase::CoupledFrictionIndex96,
      NearSingularBenchmarkCase::CoupledFrictionIndex128,
      NearSingularBenchmarkCase::CoupledFrictionIndex192,
      NearSingularBenchmarkCase::CoupledFrictionIndex256};

  for (const auto testCase : cases) {
    const auto problem = MakeNearSingularBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunNearSingularBenchmark(solver, testCase, problem)) {
        continue;
      }

      const auto name = MakeNearSingularBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunNearSingularBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterNearSingularBatchBenchmarks()
{
  constexpr std::array<NearSingularBenchmarkCase, 15> cases{
      NearSingularBenchmarkCase::Standard8,
      NearSingularBenchmarkCase::Boxed8,
      NearSingularBenchmarkCase::CoupledFrictionIndex3,
      NearSingularBenchmarkCase::CoupledFrictionIndex6,
      NearSingularBenchmarkCase::CoupledFrictionIndex9,
      NearSingularBenchmarkCase::CoupledFrictionIndex12,
      NearSingularBenchmarkCase::CoupledFrictionIndex16,
      NearSingularBenchmarkCase::CoupledFrictionIndex24,
      NearSingularBenchmarkCase::CoupledFrictionIndex32,
      NearSingularBenchmarkCase::CoupledFrictionIndex48,
      NearSingularBenchmarkCase::CoupledFrictionIndex64,
      NearSingularBenchmarkCase::CoupledFrictionIndex96,
      NearSingularBenchmarkCase::CoupledFrictionIndex128,
      NearSingularBenchmarkCase::CoupledFrictionIndex192,
      NearSingularBenchmarkCase::CoupledFrictionIndex256};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    const auto problems = MakeNearSingularBatchProblems(testCase, batchSize);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunNearSingularBatchBenchmark(
              solver, testCase, problems)) {
        continue;
      }

      const auto serialName
          = MakeNearSingularBatchSerialBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunNearSingularBatchSerialBenchmark(state, solver, testCase);
          })
          ->Arg(batchSize);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
      const auto parallelName
          = MakeNearSingularBatchParallelBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunNearSingularBatchParallelBenchmark(state, solver, testCase);
          })
          ->Arg(batchSize);
#endif
    }
  }
}

void RegisterSingularDegenerateBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 3> cases{
      SingularDegenerateBenchmarkCase::Standard16,
      SingularDegenerateBenchmarkCase::Boxed16,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex6};

  for (const auto testCase : cases) {
    const auto problem = MakeSingularDegenerateBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(
              solver, testCase, problem)) {
        continue;
      }

      const auto name = MakeSingularDegenerateBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterLargerSingularDegenerateBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 3> cases{
      SingularDegenerateBenchmarkCase::Standard32,
      SingularDegenerateBenchmarkCase::Boxed32,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex8};

  for (const auto testCase : cases) {
    const auto problem = MakeSingularDegenerateBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(
              solver, testCase, problem)) {
        continue;
      }

      const auto name
          = MakeLargerSingularDegenerateBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterStressSingularDegenerateBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 3> cases{
      SingularDegenerateBenchmarkCase::Standard64,
      SingularDegenerateBenchmarkCase::Boxed64,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex12};

  for (const auto testCase : cases) {
    const auto problem = MakeSingularDegenerateBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(
              solver, testCase, problem)) {
        continue;
      }

      const auto name
          = MakeStressSingularDegenerateBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterExtremeSingularDegenerateBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 11> cases{
      SingularDegenerateBenchmarkCase::Standard128,
      SingularDegenerateBenchmarkCase::Boxed128,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex16,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex24,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex32,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex48,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex64,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex96,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex128,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex192,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex256};

  for (const auto testCase : cases) {
    const auto problem = MakeSingularDegenerateBenchmarkProblem(testCase);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(
              solver, testCase, problem)) {
        continue;
      }

      const auto name
          = MakeExtremeSingularDegenerateBenchmarkName(testCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(), [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterSingularDegenerateFrictionIndexBatchBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 12> cases{
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex6,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex8,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex12,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex16,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex24,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex32,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex48,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex64,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex96,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex128,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex192,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex256};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    const auto problems
        = MakeSingularDegenerateFrictionIndexBatchProblems(testCase, batchSize);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBatchBenchmark(
              solver, testCase, problems)) {
        continue;
      }

      const auto serialName
          = MakeSingularDegenerateFrictionIndexBatchSerialBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateFrictionIndexBatchSerialBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
      const auto parallelName
          = MakeSingularDegenerateFrictionIndexBatchParallelBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateFrictionIndexBatchParallelBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);
#endif
    }
  }
}

void RegisterSingularDegenerateStandardBoxedBatchBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 8> cases{
      SingularDegenerateBenchmarkCase::Standard16,
      SingularDegenerateBenchmarkCase::Boxed16,
      SingularDegenerateBenchmarkCase::Standard32,
      SingularDegenerateBenchmarkCase::Boxed32,
      SingularDegenerateBenchmarkCase::Standard64,
      SingularDegenerateBenchmarkCase::Boxed64,
      SingularDegenerateBenchmarkCase::Standard128,
      SingularDegenerateBenchmarkCase::Boxed128};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    const auto problems
        = MakeSingularDegenerateStandardBoxedBatchProblems(testCase, batchSize);
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBatchBenchmark(
              solver, testCase, problems)) {
        continue;
      }

      const auto serialName
          = MakeSingularDegenerateStandardBoxedBatchSerialBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateStandardBoxedBatchSerialBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
      const auto parallelName
          = MakeSingularDegenerateStandardBoxedBatchParallelBenchmarkName(
              testCase, solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(),
          [solver, testCase](benchmark::State& state) {
            RunSingularDegenerateStandardBoxedBatchParallelBenchmark(
                state, solver, testCase);
          })
          ->Arg(batchSize);
#endif
    }
  }
}

void RegisterBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      const auto args = GetConcreteBatchBenchmarkArgs(solver, family);
      if (args.empty()) {
        continue;
      }

      const auto name = MakeBatchBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestBatchBenchmark(state, solver, family);
          });
      AddBatchBenchmarkArgs(registeredBenchmark, args);
    }
  }
}

void RegisterGroupedBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (solver.name != "Jacobi" && solver.name != "Pgs") {
        continue;
      }

      const auto args = GetConcreteGroupedBatchBenchmarkArgs(solver, family);
      if (args.empty()) {
        continue;
      }

      const auto name = MakeGroupedBatchSerialBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunGroupedBatchSerialBenchmark(state, solver, family);
          });
      AddBenchmarkArgs(registeredBenchmark, args);
    }
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
void RegisterParallelBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      const auto args = GetConcreteBatchBenchmarkArgs(solver, family);
      if (args.empty()) {
        continue;
      }

      const auto name = MakeParallelBatchBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestParallelBatchBenchmark(state, solver, family);
          });
      AddBatchBenchmarkArgs(registeredBenchmark, args);
    }
  }
}

void RegisterGroupedParallelBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (solver.name != "Jacobi" && solver.name != "Pgs") {
        continue;
      }

      const auto args = GetConcreteGroupedBatchBenchmarkArgs(solver, family);
      if (args.empty()) {
        continue;
      }

      const auto name = MakeGroupedBatchParallelBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunGroupedBatchParallelBenchmark(state, solver, family);
          });
      AddBenchmarkArgs(registeredBenchmark, args);
    }
  }
}

struct BenchmarkArgProblem
{
  int arg{0};
  LcpProblem problem;
};

std::vector<int> GetConcreteContactBenchmarkArgs(
    const dart::test::LcpSolverManifestEntry& solver,
    const std::vector<BenchmarkArgProblem>& argProblems)
{
  std::vector<int> args;
  for (const auto& argProblem : argProblems) {
    if (SolverSupportsConcreteProblem(solver, argProblem.problem)) {
      args.push_back(argProblem.arg);
    }
  }

  return args;
}

std::vector<BenchmarkArgProblem> MakeWorldContactBenchmarkArgProblems()
{
  std::vector<BenchmarkArgProblem> argProblems;
  for (const int contactCount : std::array<int, 3>{1, 2, 4}) {
    std::string errorMessage;
    auto fixture = MakeWorldContactBenchmarkProblem(contactCount, errorMessage);
    if (!fixture.has_value()) {
      continue;
    }

    argProblems.push_back(
        BenchmarkArgProblem{contactCount, std::move(fixture->problem)});
  }

  return argProblems;
}

std::vector<BenchmarkArgProblem> MakeWorldStackContactBenchmarkArgProblems()
{
  constexpr std::array<int, 17> sphereCounts{
      2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 32};

  std::vector<BenchmarkArgProblem> argProblems;
  for (const int sphereCount : sphereCounts) {
    std::string errorMessage;
    auto fixture
        = MakeWorldStackContactBenchmarkProblem(sphereCount, errorMessage);
    if (!fixture.has_value()) {
      continue;
    }

    argProblems.push_back(
        BenchmarkArgProblem{sphereCount, std::move(fixture->problem)});
  }

  return argProblems;
}

std::vector<int> GetWorldBoxContactBenchmarkArgs()
{
  return {1, 2, 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256};
}

std::vector<BenchmarkArgProblem> MakeWorldBoxContactBenchmarkArgProblems()
{
  std::vector<BenchmarkArgProblem> argProblems;
  for (const int boxCount : GetWorldBoxContactBenchmarkArgs()) {
    std::string errorMessage;
    auto fixture
        = MakeWorldBoxContactBenchmarkProblem(errorMessage, 0, boxCount);
    if (!fixture.has_value()) {
      continue;
    }

    argProblems.push_back(
        BenchmarkArgProblem{boxCount, std::move(fixture->problem)});
  }

  return argProblems;
}

struct ArticulatedContactBenchmarkProbe
{
  ArticulatedContactBenchmarkCase benchmarkCase;
  LcpProblem problem;
};

std::vector<ArticulatedContactBenchmarkProbe>
MakeArticulatedUnifiedContactSupportProbes()
{
  std::vector<ArticulatedContactBenchmarkProbe> probes;
  for (const auto benchmarkCase : std::array{
           ArticulatedContactBenchmarkCase::Ground,
           ArticulatedContactBenchmarkCase::RigidImpact,
           ArticulatedContactBenchmarkCase::CrossLinkImpact}) {
    std::string errorMessage;
    auto fixture = MakeArticulatedUnifiedContactBenchmarkProblem(
        benchmarkCase, 1, errorMessage);
    if (!fixture.has_value()) {
      continue;
    }

    probes.push_back(
        ArticulatedContactBenchmarkProbe{
            benchmarkCase, std::move(fixture->problem)});
  }

  return probes;
}

std::vector<int> GetArticulatedUnifiedContactBenchmarkArgs()
{
  return {1, 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256};
}

std::optional<WorldContactBenchmarkBatch> MakeWorldContactBatchSupportProbe(
    const WorldContactBatchKind batchKind)
{
  std::string errorMessage;
  return MakeWorldContactBenchmarkBatch(errorMessage, batchKind);
}

std::vector<BatchBenchmarkArg> GetWorldBoxContactBatchArgCandidates()
{
  constexpr int batchSize = 4;
  return {
      {1, batchSize},
      {4, batchSize},
      {8, batchSize},
      {16, batchSize},
      {24, batchSize},
      {32, batchSize},
      {48, batchSize},
      {64, batchSize},
      {96, batchSize},
      {128, batchSize},
      {192, batchSize},
      {256, batchSize}};
}

bool ShouldRunWorldBoxContactBatchArg(
    const dart::test::LcpSolverManifestEntry& solver,
    const BatchBenchmarkArg& arg)
{
  if (solver.name == "Pgs") {
    return true;
  }
  if (solver.name == "Admm" && arg.problemArg == 256) {
    return false;
  }

  return arg.problemArg == 24 || arg.problemArg == 64 || arg.problemArg == 96
         || arg.problemArg == 128 || arg.problemArg == 192
         || arg.problemArg == 256;
}

struct WorldBoxContactBatchArgProblems
{
  BatchBenchmarkArg arg;
  std::vector<LcpProblem> problems;
};

std::vector<WorldBoxContactBatchArgProblems>
MakeWorldBoxContactBatchArgProblems()
{
  std::vector<WorldBoxContactBatchArgProblems> argProblems;
  for (const auto candidate : GetWorldBoxContactBatchArgCandidates()) {
    std::string errorMessage;
    auto batch = MakeWorldBoxContactBenchmarkBatch(
        candidate.problemArg, candidate.batchSize, errorMessage);
    if (!batch.has_value()) {
      continue;
    }

    argProblems.push_back(
        WorldBoxContactBatchArgProblems{candidate, std::move(batch->problems)});
  }

  return argProblems;
}

std::vector<BatchBenchmarkArg> GetConcreteWorldBoxContactBatchArgs(
    const dart::test::LcpSolverManifestEntry& solver,
    const std::vector<WorldBoxContactBatchArgProblems>& argProblems)
{
  std::vector<BatchBenchmarkArg> args;
  for (const auto& candidate : argProblems) {
    if (!ShouldRunWorldBoxContactBatchArg(solver, candidate.arg)) {
      continue;
    }

    if (SolverSupportsConcreteProblemBatch(solver, candidate.problems)) {
      args.push_back(candidate.arg);
    }
  }

  return args;
}

void RegisterWorldContactBenchmarks()
{
  const auto argProblems = MakeWorldContactBenchmarkArgProblems();

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    const auto args = GetConcreteContactBenchmarkArgs(solver, argProblems);
    if (args.empty()) {
      continue;
    }

    const auto name = MakeWorldContactBenchmarkName(solver);
    AddBenchmarkArgs(
        benchmark::RegisterBenchmark(
            name.c_str(),
            [solver](benchmark::State& state) {
              RunWorldContactBenchmark(state, solver);
            }),
        args);
  }
}

void RegisterWorldBoxContactBenchmarks()
{
  const auto argProblems = MakeWorldBoxContactBenchmarkArgProblems();

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!SupportsDenseWorldBoxContactPatch(solver.name)) {
      continue;
    }
    const auto args = GetConcreteContactBenchmarkArgs(solver, argProblems);
    if (args.empty()) {
      continue;
    }

    const auto name = MakeWorldBoxContactBenchmarkName(solver);
    AddBenchmarkArgs(
        benchmark::RegisterBenchmark(
            name.c_str(),
            [solver](benchmark::State& state) {
              RunWorldBoxContactBenchmark(state, solver);
            }),
        args);
  }
}

void RegisterWorldStackContactBenchmarks()
{
  const auto argProblems = MakeWorldStackContactBenchmarkArgProblems();

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    const auto args = GetConcreteContactBenchmarkArgs(solver, argProblems);
    if (args.empty()) {
      continue;
    }

    const auto name = MakeWorldStackContactBenchmarkName(solver);
    AddBenchmarkArgs(
        benchmark::RegisterBenchmark(
            name.c_str(),
            [solver](benchmark::State& state) {
              RunWorldStackContactBenchmark(state, solver);
            }),
        args);
  }
}

void RegisterArticulatedUnifiedContactBenchmarks()
{
  const auto supportProbes = MakeArticulatedUnifiedContactSupportProbes();
  if (supportProbes.empty()) {
    return;
  }

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    for (const auto& supportProbe : supportProbes) {
      if (!SolverSupportsConcreteProblem(solver, supportProbe.problem)) {
        continue;
      }

      const auto name = MakeArticulatedUnifiedContactBenchmarkName(
          supportProbe.benchmarkCase, solver);
      AddBenchmarkArgs(
          benchmark::RegisterBenchmark(
              name.c_str(),
              [solver, benchmarkCase = supportProbe.benchmarkCase](
                  benchmark::State& state) {
                RunArticulatedUnifiedContactBenchmark(
                    state, solver, benchmarkCase);
              }),
          GetArticulatedUnifiedContactBenchmarkArgs());
    }
  }
}

void RegisterStaggeringContactPipelineSweepBenchmarks()
{
  for (const auto testCase : kStaggeringContactPipelineSweepCases) {
    const auto name = MakeStaggeringContactPipelineSweepBenchmarkName(testCase);
    benchmark::RegisterBenchmark(
        name.c_str(), [testCase](benchmark::State& state) {
          RunStaggeringContactPipelineSweepBenchmark(state, testCase);
        });
  }
}

void RegisterContactSolverComparisonSweepBenchmarks()
{
  for (const auto testCase : kStaggeringContactPipelineSweepCases) {
    std::string errorMessage;
    const auto fixture
        = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
    if (!fixture.has_value()) {
      continue;
    }

    for (const auto solverName : kContactComparisonSolverNames) {
      const auto* solverEntry = FindSolverManifestEntry(solverName);
      if (solverEntry == nullptr
          || !SolverSupportsConcreteProblem(*solverEntry, fixture->problem)) {
        continue;
      }

      const auto name = MakeContactSolverComparisonSweepBenchmarkName(
          *solverEntry, testCase);
      benchmark::RegisterBenchmark(
          name.c_str(),
          [solver = *solverEntry, testCase](benchmark::State& state) {
            RunContactSolverComparisonSweepBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterContactNormalStandardSweepBenchmarks()
{
  for (const auto testCase : kContactNormalStandardSweepCases) {
    std::string errorMessage;
    const auto fixture
        = MakeStaggeringContactPipelineSweepProblem(testCase, errorMessage);
    if (!fixture.has_value()) {
      continue;
    }

    const auto problem
        = MakeContactNormalStandardProblem(*fixture, errorMessage);
    if (!problem.has_value()) {
      continue;
    }

    for (const auto solverName : kContactNormalStandardSolverNames) {
      const auto* solverEntry = FindSolverManifestEntry(solverName);
      if (solverEntry == nullptr
          || !SolverSupportsConcreteProblem(*solverEntry, *problem)) {
        continue;
      }

      const auto name
          = MakeContactNormalStandardSweepBenchmarkName(*solverEntry, testCase);
      benchmark::RegisterBenchmark(
          name.c_str(),
          [solver = *solverEntry, testCase](benchmark::State& state) {
            RunContactNormalStandardSweepBenchmark(state, solver, testCase);
          });
    }
  }
}

void RegisterWorldContactBatchBenchmarks()
{
  const auto baselineProbe
      = MakeWorldContactBatchSupportProbe(WorldContactBatchKind::Baseline);
  const auto stressProbe
      = MakeWorldContactBatchSupportProbe(WorldContactBatchKind::StressStack);
  const auto pipeline32Probe = MakeWorldContactBatchSupportProbe(
      WorldContactBatchKind::ContactPipeline32);

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (baselineProbe.has_value()
        && SolverSupportsConcreteProblemBatch(
            solver, baselineProbe->problems)) {
      const auto serialName = MakeWorldContactBatchSerialBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          serialName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchSerialBenchmark(state, solver);
          });

      const auto parallelName
          = MakeWorldContactBatchParallelBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          parallelName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchParallelBenchmark(state, solver);
          });
    }

    if (solver.name != "NNCG" && stressProbe.has_value()
        && SolverSupportsConcreteProblemBatch(solver, stressProbe->problems)) {
      const auto stressSerialName
          = MakeWorldContactStressBatchSerialBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          stressSerialName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchSerialBenchmark(
                state, solver, WorldContactBatchKind::StressStack);
          });

      const auto stressParallelName
          = MakeWorldContactStressBatchParallelBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          stressParallelName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchParallelBenchmark(
                state, solver, WorldContactBatchKind::StressStack);
          });
    }

    if (pipeline32Probe.has_value()
        && SolverSupportsConcreteProblemBatch(
            solver, pipeline32Probe->problems)) {
      const auto pipeline32SerialName
          = MakeWorldContactPipeline32BatchSerialBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          pipeline32SerialName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchSerialBenchmark(
                state, solver, WorldContactBatchKind::ContactPipeline32);
          });

      const auto pipeline32ParallelName
          = MakeWorldContactPipeline32BatchParallelBenchmarkName(solver);
      benchmark::RegisterBenchmark(
          pipeline32ParallelName.c_str(), [solver](benchmark::State& state) {
            RunWorldContactBatchParallelBenchmark(
                state, solver, WorldContactBatchKind::ContactPipeline32);
          });
    }
  }
}

void RegisterWorldBoxContactBatchBenchmarks()
{
  const auto argProblems = MakeWorldBoxContactBatchArgProblems();

  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!SupportsDenseWorldBoxContactPatch(solver.name)) {
      continue;
    }
    const auto args = GetConcreteWorldBoxContactBatchArgs(solver, argProblems);
    if (args.empty()) {
      continue;
    }

    const auto serialName = MakeWorldBoxContactBatchSerialBenchmarkName(solver);
    auto* serialBatchBenchmark = benchmark::RegisterBenchmark(
        serialName.c_str(), [solver](benchmark::State& state) {
          RunWorldBoxContactBatchSerialBenchmark(state, solver);
        });
    AddBatchBenchmarkArgs(serialBatchBenchmark, args);

    const auto parallelName
        = MakeWorldBoxContactBatchParallelBenchmarkName(solver);
    auto* parallelBatchBenchmark = benchmark::RegisterBenchmark(
        parallelName.c_str(), [solver](benchmark::State& state) {
          RunWorldBoxContactBatchParallelBenchmark(state, solver);
        });
    AddBatchBenchmarkArgs(parallelBatchBenchmark, args);
  }
}
#endif

static void BM_LcpCompare_Dantzig_Scaled(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const double scale = (state.range(1) == 0) ? 1e-6 : 1e6;
  const auto problem
      = MakeScaledProblem(n, scale, 777u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Scaled"));
}

static void BM_LcpCompare_Pgs_Scaled(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  const double scale = (state.range(1) == 0) ? 1e-6 : 1e6;
  const auto problem
      = MakeScaledProblem(n, scale, 888u + static_cast<unsigned>(n));
  const auto options = MakeBenchmarkOptions(100);
  RunBenchmark<dart::math::PgsSolver>(
      state, problem, options, MakeLabel("Pgs", "Scaled"));
}

static void BM_LCP_COMPARE_SMOKE(benchmark::State& state)
{
  const auto problem = MakeStandardSpdProblem(12, 1u);
  const auto options = MakeBenchmarkOptions(30);
  RunBenchmark<dart::math::DantzigSolver>(
      state, problem, options, MakeLabel("Dantzig", "Smoke"));
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
static void BM_LcpCudaJacobiBatch_Standard(benchmark::State& state)
{
  RunCudaJacobiBatchBenchmark(state, BenchmarkProblemFamily::Standard);
}

static void BM_LcpCudaJacobiBatch_Boxed(benchmark::State& state)
{
  RunCudaJacobiBatchBenchmark(state, BenchmarkProblemFamily::Boxed);
}

static void BM_LcpCudaJacobiBatch_FrictionIndex(benchmark::State& state)
{
  RunCudaJacobiBatchBenchmark(state, BenchmarkProblemFamily::FrictionIndex);
}

static void BM_LcpCudaPgsBatch_Standard(benchmark::State& state)
{
  RunCudaPgsBatchBenchmark(state, BenchmarkProblemFamily::Standard);
}

static void BM_LcpCudaPgsBatch_Boxed(benchmark::State& state)
{
  RunCudaPgsBatchBenchmark(state, BenchmarkProblemFamily::Boxed);
}

static void BM_LcpCudaPgsBatch_FrictionIndex(benchmark::State& state)
{
  RunCudaPgsBatchBenchmark(state, BenchmarkProblemFamily::FrictionIndex);
}

static void BM_LcpCudaJacobiGroupedBatch_Standard(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(state, BenchmarkProblemFamily::Standard, false);
}

static void BM_LcpCudaJacobiGroupedBatch_Boxed(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(state, BenchmarkProblemFamily::Boxed, false);
}

static void BM_LcpCudaJacobiGroupedBatch_FrictionIndex(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(
      state, BenchmarkProblemFamily::FrictionIndex, false);
}

static void BM_LcpCudaPgsGroupedBatch_Standard(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(state, BenchmarkProblemFamily::Standard, true);
}

static void BM_LcpCudaPgsGroupedBatch_Boxed(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(state, BenchmarkProblemFamily::Boxed, true);
}

static void BM_LcpCudaPgsGroupedBatch_FrictionIndex(benchmark::State& state)
{
  RunCudaGroupedBatchBenchmark(
      state, BenchmarkProblemFamily::FrictionIndex, true);
}
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION                                         \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
static void BM_LcpCudaJacobiWorldContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldContactBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldContactBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiWorldBoxContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactGroupedBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactGroupedBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiWorldStackContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldStackContactBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldStackContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldStackContactBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiWorldContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldContactGroupedBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldContactGroupedBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiWorldStackContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldStackContactGroupedBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsWorldStackContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldStackContactGroupedBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaArticulatedUnifiedContactGroupedBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaArticulatedUnifiedContactGroupedBatchBenchmark(state, true);
}

static void BM_LcpCudaJacobiMixedContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaMixedContactGroupedBatchBenchmark(state, false);
}

static void BM_LcpCudaPgsMixedContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaMixedContactGroupedBatchBenchmark(state, true);
}
#endif

const bool kManifestBenchmarksRegistered = [] {
  RegisterManifestBenchmarks();
  RegisterActiveSetTransitionBenchmarks();
  RegisterActiveFrictionIndexContactBenchmarks();
  RegisterPgsRelaxationSweepBenchmarks();
  RegisterSymmetricPsorRelaxationSweepBenchmarks();
  RegisterRedBlackGaussSeidelRelaxationSweepBenchmarks();
  RegisterBoxedSsnLineSearchSweepBenchmarks();
  RegisterPivotingScaleSweepBenchmarks();
  RegisterBlockPartitionSweepBenchmarks();
  RegisterApgdRestartSweepBenchmarks();
  RegisterTgsIterationBudgetSweepBenchmarks();
  RegisterNncgPgsIterationsSweepBenchmarks();
  RegisterSubspacePgsIterationsSweepBenchmarks();
  RegisterShockPropagationLayerSweepBenchmarks();
  RegisterMprgpSpdCheckSweepBenchmarks();
  RegisterInteriorPointPathSweepBenchmarks();
  RegisterAdmmRhoSweepBenchmarks();
  RegisterSapRegularizationSweepBenchmarks();
  RegisterNewtonWarmStartBenchmarks();
  RegisterLargerActiveSetTransitionBenchmarks();
  RegisterStressActiveSetTransitionBenchmarks();
  RegisterExtremeActiveSetTransitionBenchmarks();
  RegisterProductionActiveSetTransitionBenchmarks();
  RegisterProductionActiveSetTransitionBatchBenchmarks();
  RegisterMildIllConditionedBenchmarks();
  RegisterMildIllConditionedBatchBenchmarks();
  RegisterNearSingularBenchmarks();
  RegisterNearSingularBatchBenchmarks();
  RegisterSingularDegenerateBenchmarks();
  RegisterLargerSingularDegenerateBenchmarks();
  RegisterStressSingularDegenerateBenchmarks();
  RegisterExtremeSingularDegenerateBenchmarks();
  RegisterSingularDegenerateFrictionIndexBatchBenchmarks();
  RegisterSingularDegenerateStandardBoxedBatchBenchmarks();
  RegisterBatchBenchmarks();
  RegisterGroupedBatchBenchmarks();
#if DART_BM_LCP_COMPARE_HAS_SIMULATION
  RegisterParallelBatchBenchmarks();
  RegisterGroupedParallelBatchBenchmarks();
  RegisterWorldContactBenchmarks();
  RegisterWorldBoxContactBenchmarks();
  RegisterWorldStackContactBenchmarks();
  RegisterArticulatedUnifiedContactBenchmarks();
  RegisterStaggeringContactPipelineSweepBenchmarks();
  RegisterContactSolverComparisonSweepBenchmarks();
  RegisterContactNormalStandardSweepBenchmarks();
  RegisterWorldContactBatchBenchmarks();
  RegisterWorldBoxContactBatchBenchmarks();
#endif
  return true;
}();

} // namespace

BENCHMARK(BM_LcpCompare_Dantzig_Scaled)->Args({12, 0})->Args({12, 1});
BENCHMARK(BM_LcpCompare_Pgs_Scaled)->Args({12, 0})->Args({12, 1});

BENCHMARK(BM_LcpValidation_Serial_FrictionIndex)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpValidation_Threaded_FrictionIndex)->Arg(16)->Arg(64);
BENCHMARK(BM_LcpJacobiSolverThreading_Standard)
    ->Args({128, 1})
    ->Args({128, 8})
    ->Args({512, 1})
    ->Args({512, 8});
BENCHMARK(BM_LcpJacobiSolverThreadingBanded_Standard)
    ->Args({512, 1})
    ->Args({512, 4})
    ->Args({512, 8})
    ->Args({1024, 1})
    ->Args({1024, 4})
    ->Args({1024, 8})
    ->Args({1024, 16})
    ->Args({2048, 1})
    ->Args({2048, 8})
    ->Args({2048, 16})
    ->Args({2048, 32})
    ->Args({4096, 1})
    ->Args({4096, 8})
    ->Args({4096, 16})
    ->Args({4096, 32})
    ->Args({8192, 1})
    ->Args({8192, 32});
BENCHMARK(BM_LcpRedBlackGaussSeidelSolverThreadingBanded_Standard)
    ->Args({128, 1})
    ->Args({128, 4})
    ->Args({512, 1})
    ->Args({512, 4})
    ->Args({512, 8})
    ->Args({1024, 1})
    ->Args({1024, 4})
    ->Args({1024, 8})
    ->Args({2048, 1})
    ->Args({2048, 4})
    ->Args({2048, 8})
    ->Args({4096, 1})
    ->Args({4096, 32})
    ->Args({8192, 1})
    ->Args({8192, 32});
BENCHMARK(BM_LcpBlockedJacobiSolverThreadingBanded_Standard)
    ->Args({128, 1})
    ->Args({128, 4})
    ->Args({512, 1})
    ->Args({512, 4})
    ->Args({512, 8})
    ->Args({1024, 1})
    ->Args({1024, 4})
    ->Args({1024, 8})
    ->Args({2048, 1})
    ->Args({2048, 4})
    ->Args({2048, 8})
    ->Args({4096, 1})
    ->Args({4096, 32})
    ->Args({8192, 1})
    ->Args({8192, 32});

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
BENCHMARK(BM_LcpCudaJacobiBatch_Standard)
    ->Args({24, 4})
    ->Args({48, 4})
    ->Args({96, 4})
    ->Args({128, 4})
    ->Args({192, 4})
    ->Args({256, 4});
BENCHMARK(BM_LcpCudaJacobiBatch_Boxed)
    ->Args({24, 4})
    ->Args({48, 4})
    ->Args({96, 4})
    ->Args({128, 4})
    ->Args({192, 4})
    ->Args({256, 4});
BENCHMARK(BM_LcpCudaJacobiBatch_FrictionIndex)
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({32, 4})
    ->Args({48, 4})
    ->Args({64, 4})
    ->Args({96, 4});
BENCHMARK(BM_LcpCudaPgsBatch_Standard)
    ->Args({24, 4})
    ->Args({48, 4})
    ->Args({96, 4})
    ->Args({128, 4})
    ->Args({192, 4})
    ->Args({256, 4});
BENCHMARK(BM_LcpCudaPgsBatch_Boxed)
    ->Args({24, 4})
    ->Args({48, 4})
    ->Args({96, 4})
    ->Args({128, 4})
    ->Args({192, 4})
    ->Args({256, 4});
BENCHMARK(BM_LcpCudaPgsBatch_FrictionIndex)
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({32, 4})
    ->Args({48, 4})
    ->Args({64, 4})
    ->Args({96, 4});
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_Standard)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_Boxed)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_FrictionIndex)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_Standard)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_Boxed)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_FrictionIndex)->Arg(2)->Arg(3);
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION                                         \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA
BENCHMARK(BM_LcpCudaJacobiWorldContactBatch_FrictionIndex)
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4});
BENCHMARK(BM_LcpCudaPgsWorldContactBatch_FrictionIndex)
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4});
BENCHMARK(BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex)
    ->Args({1, 4})
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4})
    ->Args({48, 4})
    ->Args({64, 4})
    ->Args({96, 4})
    ->Args({128, 1})
    ->Args({128, 4})
    ->Args({192, 4})
    ->Args({256, 4});
BENCHMARK(BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex)
    ->Args({1, 4})
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4})
    ->Args({48, 4})
    ->Args({64, 4})
    ->Args({96, 4})
    ->Args({128, 1})
    ->Args({128, 4})
    ->Args({192, 1});
BENCHMARK(BM_LcpCudaJacobiWorldBoxContactGroupedBatch_FrictionIndex)
    ->Args({2})
    ->Args({3});
BENCHMARK(BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex)
    ->Args({2})
    ->Args({3});
BENCHMARK(BM_LcpCudaJacobiWorldStackContactBatch_FrictionIndex)
    ->Args({5, 4})
    ->Args({6, 4})
    ->Args({7, 4})
    ->Args({8, 4})
    ->Args({9, 4})
    ->Args({10, 4})
    ->Args({11, 4})
    ->Args({12, 4})
    ->Args({13, 4})
    ->Args({14, 4})
    ->Args({15, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4});
BENCHMARK(BM_LcpCudaPgsWorldStackContactBatch_FrictionIndex)
    ->Args({5, 4})
    ->Args({6, 4})
    ->Args({7, 4})
    ->Args({8, 4})
    ->Args({9, 4})
    ->Args({10, 4})
    ->Args({11, 4})
    ->Args({12, 4})
    ->Args({13, 4})
    ->Args({14, 4})
    ->Args({15, 4})
    ->Args({16, 4})
    ->Args({24, 4})
    ->Args({32, 4});
BENCHMARK(BM_LcpCudaJacobiWorldContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaPgsWorldContactGroupedBatch_FrictionIndex)->Arg(2)->Arg(3);
BENCHMARK(BM_LcpCudaJacobiWorldStackContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaPgsWorldStackContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaJacobiMixedContactGroupedBatch_FrictionIndex)
    ->Arg(2)
    ->Arg(3);
BENCHMARK(BM_LcpCudaPgsMixedContactGroupedBatch_FrictionIndex)->Arg(2)->Arg(3);
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION
BENCHMARK(BM_LcpWorldContactAssembly_BoxedLcp)->Arg(1)->Arg(2)->Arg(4);
BENCHMARK(BM_LcpWorldStackContactAssembly_BoxedLcp)
    ->Arg(2)
    ->Arg(3)
    ->Arg(4)
    ->Arg(5)
    ->Arg(6)
    ->Arg(7)
    ->Arg(8)
    ->Arg(9)
    ->Arg(10)
    ->Arg(11)
    ->Arg(12)
    ->Arg(13)
    ->Arg(14)
    ->Arg(15)
    ->Arg(16)
    ->Arg(24)
    ->Arg(32);
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({3, 200})->Args({3, 500});
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({4, 200});
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({5, 500});
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({6, 1000});
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)
    ->Args({7, 1})
    ->Args({8, 1})
    ->Args({9, 1})
    ->Args({10, 1})
    ->Args({11, 1})
    ->Args({12, 1})
    ->Args({13, 1})
    ->Args({14, 1})
    ->Args({15, 1})
    ->Args({16, 1})
    ->Args({24, 1})
    ->Args({32, 1});
BENCHMARK(BM_LcpWorldSeparatedStep_BoxedLcp)
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200})
    ->Args({24, 200})
    ->Args({32, 200});
BENCHMARK(BM_LcpWorldBoxStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({2, 200})
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 500})
    ->Args({24, 2000})
    ->Args({32, 4000})
    ->Args({48, 4000})
    ->Args({64, 1})
    ->Args({64, 75})
    ->Args({96, 1})
    ->Args({96, 75})
    ->Args({128, 1})
    ->Args({128, 75})
    ->Args({144, 1})
    ->Args({144, 75})
    ->Args({192, 1})
    ->Args({256, 1});
BENCHMARK(BM_LcpWorldBilliardsStep_BoxedLcp)
    ->Args({1, 1})
    ->Args({4, 1})
    ->Args({8, 1});
BENCHMARK(BM_LcpWorldCardPileStep_BoxedLcp)
    ->Args({4, 200})
    ->Args({7, 200})
    ->Args({12, 200});
BENCHMARK(BM_LcpWorldArticulatedGroundStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200})
    ->Args({24, 200})
    ->Args({32, 200})
    ->Args({64, 200})
    ->Args({96, 200})
    ->Args({128, 200})
    ->Args({192, 200})
    ->Args({256, 200})
    ->Args({384, 200})
    ->Args({512, 200})
    ->Args({768, 200})
    ->Args({1024, 200})
    ->Args({1536, 200})
    ->Args({2048, 200});
BENCHMARK(BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp)
    ->Args({1, 1})
    ->Args({4, 1})
    ->Args({8, 1})
    ->Args({16, 1})
    ->Args({16, 200})
    ->Args({24, 1})
    ->Args({32, 1})
    ->Args({32, 200})
    ->Args({64, 1})
    ->Args({64, 200})
    ->Args({96, 1})
    ->Args({96, 200})
    ->Args({128, 1})
    ->Args({128, 200})
    ->Args({192, 1})
    ->Args({192, 200})
    ->Args({256, 1})
    ->Args({256, 200})
    ->Args({384, 1})
    ->Args({384, 200})
    ->Args({512, 1})
    ->Args({512, 200})
    ->Args({768, 1})
    ->Args({768, 200})
    ->Args({1024, 1})
    ->Args({1024, 200})
    ->Args({1536, 1})
    ->Args({1536, 200})
    ->Args({2048, 1})
    ->Args({2048, 200});
BENCHMARK(BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp)
    ->Args({1, 1})
    ->Args({4, 1})
    ->Args({8, 1})
    ->Args({16, 1})
    ->Args({16, 200})
    ->Args({24, 1})
    ->Args({32, 1})
    ->Args({32, 200})
    ->Args({64, 1})
    ->Args({64, 200})
    ->Args({96, 1})
    ->Args({96, 200})
    ->Args({128, 1})
    ->Args({128, 200})
    ->Args({192, 1})
    ->Args({192, 200})
    ->Args({256, 1})
    ->Args({256, 200})
    ->Args({384, 1})
    ->Args({384, 200})
    ->Args({512, 1})
    ->Args({512, 200})
    ->Args({768, 1})
    ->Args({768, 200})
    ->Args({1024, 1})
    ->Args({1024, 200})
    ->Args({1536, 1})
    ->Args({1536, 200})
    ->Args({2048, 1})
    ->Args({2048, 200});
BENCHMARK(BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200})
    ->Args({24, 200})
    ->Args({32, 200})
    ->Args({64, 200})
    ->Args({96, 200})
    ->Args({128, 200})
    ->Args({192, 200})
    ->Args({256, 200})
    ->Args({384, 200})
    ->Args({512, 200})
    ->Args({768, 200})
    ->Args({1024, 200})
    ->Args({1536, 200})
    ->Args({2048, 200});
#endif

BENCHMARK(BM_LCP_COMPARE_SMOKE);
