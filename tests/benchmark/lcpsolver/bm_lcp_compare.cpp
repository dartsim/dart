/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Solver-agnostic benchmark harness for LCP comparisons.
 */

#include "tests/common/lcpsolver/lcp_solver_manifest.hpp"
#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#ifndef DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
  #define DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL 0
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
#ifndef DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
  #define DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA 0
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
  #include <dart/simulation/experimental/body/collision_shape.hpp>
  #include <dart/simulation/experimental/body/contact.hpp>
  #include <dart/simulation/experimental/body/rigid_body.hpp>
  #include <dart/simulation/experimental/body/rigid_body_options.hpp>
  #include <dart/simulation/experimental/comps/multibody.hpp>
  #include <dart/simulation/experimental/compute/compute_graph.hpp>
  #include <dart/simulation/experimental/compute/multibody_dynamics.hpp>
  #include <dart/simulation/experimental/compute/parallel_executor.hpp>
  #include <dart/simulation/experimental/compute/unified_constraint.hpp>
  #include <dart/simulation/experimental/detail/boxed_lcp_contact.hpp>
  #include <dart/simulation/experimental/detail/entity_conversion.hpp>
  #include <dart/simulation/experimental/detail/world_registry_access.hpp>
  #include <dart/simulation/experimental/multibody/joint.hpp>
  #include <dart/simulation/experimental/multibody/link.hpp>
  #include <dart/simulation/experimental/multibody/multibody.hpp>
  #include <dart/simulation/experimental/world.hpp>
  #include <dart/simulation/experimental/world_options.hpp>
#endif
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
  #include <dart/simulation/experimental/compute/cuda/lcp_jacobi_batch_cuda.cuh>
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
    == 24);
static_assert(
    dart::test::countSolversSupporting(dart::test::LcpProblemSupport::Boxed)
    == 16);
static_assert(
    dart::test::countSolversSupporting(
        dart::test::LcpProblemSupport::FrictionIndex)
    == 16);

namespace {

using dart::math::LcpOptions;
using dart::math::LcpProblem;
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
namespace compute = dart::simulation::experimental::compute;
namespace sx = dart::simulation::experimental;
#endif
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
namespace cuda_compute = dart::simulation::experimental::compute::cuda;
#endif

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
    const int numContacts, const unsigned seed, const bool coupled)
{
  const int n = 3 * numContacts;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd diag(n);
  for (int contact = 0; contact < numContacts; ++contact) {
    const double contactScale
        = 1.0 + 0.1 * static_cast<double>((seed + contact) % 7);
    diag[3 * contact + 0] = contactScale;
    diag[3 * contact + 1] = contactScale * 1e4;
    diag[3 * contact + 2] = contactScale * 1e8;
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

LcpProblem MakeScaledProblem(int n, double scale, unsigned seed)
{
  auto problem = MakeStandardSpdProblem(n, seed);
  problem.A *= scale;
  problem.b *= scale;
  return problem;
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  StressStack
};

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
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));
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

std::optional<WorldContactBenchmarkBatch> MakeWorldContactBenchmarkBatch(
    std::string& errorMessage,
    WorldContactBatchKind batchKind = WorldContactBatchKind::Baseline)
{
  constexpr std::array<int, 3> kSeparatedContactCounts{1, 2, 4};
  constexpr std::array<int, 2> kBaselineStackSphereCounts{2, 3};
  constexpr std::array<int, 4> kStressStackSphereCounts{2, 3, 4, 5};

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(
      kSeparatedContactCounts.size()
      + (batchKind == WorldContactBatchKind::StressStack
             ? kStressStackSphereCounts.size()
             : kBaselineStackSphereCounts.size()));

  auto appendProblem = [&batch](WorldContactBenchmarkProblem& fixture) {
    batch.totalProblemSize += fixture.problem.b.size();
    batch.totalContactCount += fixture.contactCount;
    batch.totalBodyCount += fixture.bodyCount;
    batch.problems.push_back(std::move(fixture.problem));
  };

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

  #if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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
    int batchSize, std::string& errorMessage)
{
  if (batchSize <= 0) {
    errorMessage = "dense box-contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBenchmarkBatch batch;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  Eigen::Index expectedRows = 0;
  for (const int i : std::views::iota(0, batchSize)) {
    auto fixture = MakeWorldBoxContactBenchmarkProblem(errorMessage, i);
    if (!fixture.has_value()) {
      return std::nullopt;
    }
    if (fixture->contactCount < 4u) {
      errorMessage = "dense box-contact batch lost face contact coverage";
      return std::nullopt;
    }
    if (expectedRows == 0) {
      expectedRows = fixture->problem.b.size();
    } else if (fixture->problem.b.size() != expectedRows) {
      errorMessage = "dense box-contact batch problem shape changed";
      return std::nullopt;
    }
    if (fixture->problem.b.size()
        != static_cast<Eigen::Index>(3 * fixture->contactCount)) {
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

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(linkCount))));
  constexpr double kSpacing = 1.5;
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

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(24.0, 24.0, 0.5)));

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(chainCount))));
  constexpr double kSpacing = 1.5;
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

bool SupportsDenseWorldBoxContactPatch(std::string_view solverName)
{
  return solverName == "Pgs" || solverName == "RedBlackGaussSeidel"
         || solverName == "NNCG" || solverName == "Apgd" || solverName == "Tgs"
         || solverName == "Admm";
}

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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  state.counters["has_simulation_experimental"]
      = DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL ? 1.0 : 0.0;
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

static void BM_LcpJacobiSolverThreading_Standard(benchmark::State& state)
{
  const int problemSize = static_cast<int>(state.range(0));
  const int requestedWorkerCount = static_cast<int>(state.range(1));
  const int workerCount = std::max(
      1, std::min(requestedWorkerCount, static_cast<int>(problemSize)));

  const auto problem = MakeStandardSpdProblem(
      problemSize, 60'001u + static_cast<unsigned>(problemSize));
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
          workerCount == 1 ? "JacobiSerial" : "JacobiThreaded", "Standard"));

  state.counters["worker_count"] = workerCount;
  state.counters["solver_internal_threads"] = workerCount;
}

enum class BenchmarkProblemFamily
{
  Standard,
  Boxed,
  FrictionIndex
};

enum class NewtonWarmStartMode
{
  None,
  Pgs,
  GradientDescent,
  PgsThenGradient
};

constexpr int kNewtonWarmStartPgsIterations = 5;
constexpr int kNewtonWarmStartGradientIterations = 5;

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
  StrongCoupledFrictionIndex16,
  StrongCoupledFrictionIndex24,
  StrongerCoupledFrictionIndex16,
  StrongerCoupledFrictionIndex24
};

enum class NearSingularBenchmarkCase
{
  Standard8,
  Boxed8,
  CoupledFrictionIndex3,
  CoupledFrictionIndex6,
  CoupledFrictionIndex9,
  CoupledFrictionIndex12
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
  CoupledFrictionIndex16
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
  CoupledFrictionIndex32
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

dart::test::LcpProblemSupport getProblemSupport(BenchmarkProblemFamily family)
{
  switch (family) {
    case BenchmarkProblemFamily::Standard:
      return dart::test::LcpProblemSupport::Standard;
    case BenchmarkProblemFamily::Boxed:
      return dart::test::LcpProblemSupport::Boxed;
    case BenchmarkProblemFamily::FrictionIndex:
      return dart::test::LcpProblemSupport::FrictionIndex;
  }

  return dart::test::LcpProblemSupport::Standard;
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
  }

  return "Unknown";
}

dart::test::LcpProblemSupport getLargerActiveSetTransitionProblemSupport(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
      return dart::test::LcpProblemSupport::Standard;
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return dart::test::LcpProblemSupport::Boxed;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return dart::test::LcpProblemSupport::FrictionIndex;
  }

  return dart::test::LcpProblemSupport::Standard;
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
                    CoupledFrictionIndex32;
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

LcpProblem MakeLargerActiveSetTransitionBenchmarkProblem(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::Standard32:
      return MakeStandardActiveSetTransitionProblem(32, 21'032u);
    case LargerActiveSetTransitionBenchmarkCase::Boxed32:
      return MakeBoxedActiveSetTransitionProblem(32, 22'032u);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex8:
      return MakeFrictionIndexActiveSetTransitionProblem(8, 23'008u);
    case LargerActiveSetTransitionBenchmarkCase::Standard64:
      return MakeStandardActiveSetTransitionProblem(64, 21'064u);
    case LargerActiveSetTransitionBenchmarkCase::Boxed64:
      return MakeBoxedActiveSetTransitionProblem(64, 22'064u);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex12:
      return MakeFrictionIndexActiveSetTransitionProblem(12, 23'012u);
    case LargerActiveSetTransitionBenchmarkCase::Standard128:
      return MakeStandardActiveSetTransitionProblem(128, 21'128u);
    case LargerActiveSetTransitionBenchmarkCase::Boxed128:
      return MakeBoxedActiveSetTransitionProblem(128, 22'128u);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex16:
      return MakeFrictionIndexActiveSetTransitionProblem(16, 23'016u);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return MakeFrictionIndexActiveSetTransitionProblem(24, 23'024u, 2.0);
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return MakeFrictionIndexActiveSetTransitionProblem(32, 23'032u, 4.0);
  }

  return MakeStandardActiveSetTransitionProblem(32, 21'032u);
}

double getLargerActiveSetTransitionCouplingScale(
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  switch (testCase) {
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24:
      return 2.0;
    case LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32:
      return 4.0;
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
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      return "StrongCoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      return "StrongCoupledFrictionIndex24";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      return "StrongerCoupledFrictionIndex16";
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return "StrongerCoupledFrictionIndex24";
  }

  return "Unknown";
}

dart::test::LcpProblemSupport getMildIllConditionedProblemSupport(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::Standard32:
      return dart::test::LcpProblemSupport::Standard;
    case MildIllConditionedBenchmarkCase::Boxed16:
      return dart::test::LcpProblemSupport::Boxed;
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return dart::test::LcpProblemSupport::FrictionIndex;
  }

  return dart::test::LcpProblemSupport::Standard;
}

template <std::size_t N>
bool SolverNameIn(
    const dart::test::LcpSolverManifestEntry& solver,
    const std::array<std::string_view, N>& names)
{
  return std::find(names.begin(), names.end(), solver.name) != names.end();
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
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex24;
}

int getMildIllConditionedContactCount(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
      return 8;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
      return 6;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
      return 12;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      return 16;
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return 24;
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
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex16
         || testCase
                == MildIllConditionedBenchmarkCase::
                    StrongerCoupledFrictionIndex24;
}

double getMildIllConditionedCouplingScale(
    const MildIllConditionedBenchmarkCase testCase)
{
  switch (testCase) {
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      return 4.0;
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return 8.0;
    case MildIllConditionedBenchmarkCase::Standard32:
    case MildIllConditionedBenchmarkCase::Boxed16:
    case MildIllConditionedBenchmarkCase::FrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex6:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex8:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex12:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex16:
    case MildIllConditionedBenchmarkCase::CoupledFrictionIndex24:
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
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(16, 28'016u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(24, 28'024u, true, 4.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16:
      return MakeMildIllConditionedFrictionIndexProblem(16, 29'016u, true, 8.0);
    case MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24:
      return MakeMildIllConditionedFrictionIndexProblem(24, 29'024u, true, 8.0);
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
  }

  return "Unknown";
}

dart::test::LcpProblemSupport getNearSingularProblemSupport(
    const NearSingularBenchmarkCase testCase)
{
  switch (testCase) {
    case NearSingularBenchmarkCase::Standard8:
      return dart::test::LcpProblemSupport::Standard;
    case NearSingularBenchmarkCase::Boxed8:
      return dart::test::LcpProblemSupport::Boxed;
    case NearSingularBenchmarkCase::CoupledFrictionIndex3:
    case NearSingularBenchmarkCase::CoupledFrictionIndex6:
    case NearSingularBenchmarkCase::CoupledFrictionIndex9:
    case NearSingularBenchmarkCase::CoupledFrictionIndex12:
      return dart::test::LcpProblemSupport::FrictionIndex;
  }

  return dart::test::LcpProblemSupport::Standard;
}

bool isNearSingularFrictionIndexCase(const NearSingularBenchmarkCase testCase)
{
  return testCase == NearSingularBenchmarkCase::CoupledFrictionIndex3
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex6
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex9
         || testCase == NearSingularBenchmarkCase::CoupledFrictionIndex12;
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
  }

  return MakeNearSingularStandardProblem(8, 14'008u);
}

LcpProblem MakeSingularDegenerateStandardProblem(const int n)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    if (i % 4 == 0) {
      w[i] = 0.25 + 0.02 * static_cast<double>(i + 1);
    } else {
      A(i, i) = 1.0 + 0.15 * static_cast<double>(i % 7);
      x[i] = 0.08 + 0.03 * static_cast<double>((i % 5) + 1);
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

LcpProblem MakeSingularDegenerateBoxedProblem(const int n)
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
  return LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

LcpProblem MakeSingularDegenerateFrictionIndexProblem(const int numContacts)
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
      const double value = magnitude * BenchmarkSignedUnitValue(r, c, 31'006u);
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
  }

  return "Unknown";
}

dart::test::LcpProblemSupport getSingularDegenerateProblemSupport(
    const SingularDegenerateBenchmarkCase testCase)
{
  switch (testCase) {
    case SingularDegenerateBenchmarkCase::Standard16:
    case SingularDegenerateBenchmarkCase::Standard32:
    case SingularDegenerateBenchmarkCase::Standard64:
    case SingularDegenerateBenchmarkCase::Standard128:
      return dart::test::LcpProblemSupport::Standard;
    case SingularDegenerateBenchmarkCase::Boxed16:
    case SingularDegenerateBenchmarkCase::Boxed32:
    case SingularDegenerateBenchmarkCase::Boxed64:
    case SingularDegenerateBenchmarkCase::Boxed128:
      return dart::test::LcpProblemSupport::Boxed;
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex6:
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex8:
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex12:
    case SingularDegenerateBenchmarkCase::CoupledFrictionIndex16:
      return dart::test::LcpProblemSupport::FrictionIndex;
  }

  return dart::test::LcpProblemSupport::Standard;
}

bool isSingularDegenerateFrictionIndexCase(
    const SingularDegenerateBenchmarkCase testCase)
{
  return testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex6
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex8
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex12
         || testCase == SingularDegenerateBenchmarkCase::CoupledFrictionIndex16;
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
  }

  return MakeSingularDegenerateStandardProblem(16);
}

int getBenchmarkMaxIterations(const dart::test::LcpSolverManifestEntry& solver)
{
  if (solver.name == "Direct") {
    return 1;
  }

  if (solver.name == "InteriorPoint" || solver.name == "MinimumMapNewton"
      || solver.name == "FischerBurmeisterNewton"
      || solver.name == "PenalizedFischerBurmeisterNewton"
      || solver.name == "Sap" || solver.name == "BoxedSemiSmoothNewton") {
    return 50;
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
  dart::math::ShockPropagationSolver::Parameters shockPropagationParams;
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
  } else if (solver.name == "SubspaceMinimization") {
    storage.subspaceParams.pgsIterations = 5;
    storage.options.customOptions = &storage.subspaceParams;
  } else if (solver.name == "PenalizedFischerBurmeisterNewton") {
    storage.penalizedFischerBurmeisterParams.lambda = 1.0;
    storage.options.customOptions = &storage.penalizedFischerBurmeisterParams;
  } else if (solver.name == "ShockPropagation") {
    ConfigureShockPropagationParameters(
        problem, storage.shockPropagationParams);
    storage.options.customOptions = &storage.shockPropagationParams;
    storage.hasShockPropagationParams = true;
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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
  std::vector<std::vector<LcpProblem>> problemGroups;
  std::vector<cuda_compute::LcpBatchCudaProblem> packets;
  int problemCount{0};
  std::size_t totalProblemSize{0};
  std::size_t minProblemSize{0};
  std::size_t maxProblemSize{0};
  int totalContactCount{0};
  int minContactCount{0};
  int maxContactCount{0};
};

std::optional<CudaGroupedBenchmarkBatch> MakeGroupedCudaBenchmarkBatch(
    BenchmarkProblemFamily family,
    int variantsPerProblemArg,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerProblemArg <= 0) {
    errorMessage = "variants per CUDA synthetic group must be positive";
    return std::nullopt;
  }

  CudaGroupedBenchmarkBatch grouped;
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

    grouped.packets.push_back(MakeCudaBatchProblem(problems, iterations));
    grouped.problemGroups.push_back(std::move(problems));
  };

  switch (family) {
    case BenchmarkProblemFamily::Standard:
    case BenchmarkProblemFamily::Boxed:
      for (const int rows : std::array<int, 3>{16, 32, 48}) {
        appendGroup(rows);
      }
      break;
    case BenchmarkProblemFamily::FrictionIndex:
      for (const int contactCount : std::array<int, 3>{4, 8, 16}) {
        appendGroup(contactCount);
      }
      break;
  }

  return grouped;
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

  constexpr std::array<int, 5> kContactCounts{1, 2, 4, 8, 16};

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

  constexpr std::array<int, 3> kBoxCounts{1, 2, 4};

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

  constexpr std::array<int, 4> kSphereCounts{2, 3, 4, 5};

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

  constexpr std::array<int, 2> kContactCounts{1, 4};
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
      10 * static_cast<std::size_t>(variantsPerScenario));

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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
void AddCudaGroupedBenchmarkCounters(
    benchmark::State& state,
    const BatchBenchmarkCounters& counters,
    const CudaGroupedBenchmarkBatch& batch,
    const std::string& label)
{
  state.counters["batch_size"] = batch.problemCount;
  state.counters["cuda_group_count"]
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
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
          std::string(getProblemFamilyName(family))));

  if (family == BenchmarkProblemFamily::FrictionIndex) {
    state.counters["contact_count"] = problemArg;
  }
  if (storage.hasShockPropagationParams) {
    AddShockPropagationCounters(state, storage.shockPropagationParams);
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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

  const int contactCount = getLargerActiveSetTransitionContactCount(testCase);
  const double couplingScale
      = getLargerActiveSetTransitionCouplingScale(testCase);
  const unsigned seedBase
      = (testCase
         == LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32)
            ? 23'032u
            : 23'024u;

  for (const int i : std::views::iota(0, batchSize)) {
    problems.push_back(MakeFrictionIndexActiveSetTransitionProblem(
        contactCount, seedBase + static_cast<unsigned>(i), couplingScale));
  }

  return problems;
}

void AddProductionActiveSetTransitionBatchCounters(
    benchmark::State& state,
    const LargerActiveSetTransitionBenchmarkCase testCase,
    const int batchSize)
{
  const int contactCount = getLargerActiveSetTransitionContactCount(testCase);
  state.counters["active_set_transition"] = 1.0;
  state.counters["production_active_set_transition_batch"] = 1.0;
  state.counters["contact_count"] = static_cast<double>(contactCount);
  state.counters["total_contact_count"]
      = static_cast<double>(contactCount * batchSize);
  state.counters["coupled"] = 1.0;
  state.counters["coupling_scale"]
      = getLargerActiveSetTransitionCouplingScale(testCase);
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
    const MildIllConditionedBenchmarkCase testCase)
{
  storage.options.maxIterations
      = std::max(storage.options.maxIterations, 20000);
  storage.options.absoluteTolerance = 1e-4;
  storage.options.relativeTolerance = 5e-3;
  storage.options.complementarityTolerance
      = isMildIllConditionedFrictionIndexCase(testCase) ? 2e-2 : 5e-3;
  storage.options.earlyTermination = true;
}

void RunMildIllConditionedBenchmark(
    benchmark::State& state,
    const dart::test::LcpSolverManifestEntry& solverEntry,
    MildIllConditionedBenchmarkCase testCase)
{
  const auto problem = MakeMildIllConditionedBenchmarkProblem(testCase);
  SolverBenchmarkOptions storage;
  ConfigureSolverBenchmarkOptions(storage, solverEntry, problem);
  ConfigureMildIllConditionedBenchmarkOptions(storage, testCase);

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
}

std::vector<LcpProblem> MakeMildIllConditionedBatchProblems(
    const MildIllConditionedBenchmarkCase testCase, const int batchSize)
{
  std::vector<LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));

  const int contactCount = getMildIllConditionedContactCount(testCase);
  const double couplingScale = getMildIllConditionedCouplingScale(testCase);
  const unsigned seedBase
      = (testCase
         == MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24)
            ? 29'024u
            : 29'016u;

  for (const int i : std::views::iota(0, batchSize)) {
    problems.push_back(MakeMildIllConditionedFrictionIndexProblem(
        contactCount,
        seedBase + static_cast<unsigned>(i),
        true,
        couplingScale));
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
  state.counters["contact_count"] = static_cast<double>(contactCount);
  state.counters["total_contact_count"]
      = static_cast<double>(contactCount * batchSize);
  state.counters["coupled"] = 1.0;
  state.counters["coupling_scale"]
      = getMildIllConditionedCouplingScale(testCase);
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
  ConfigureMildIllConditionedBenchmarkOptions(storage, testCase);

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
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  ConfigureMildIllConditionedBenchmarkOptions(fixture.storage, testCase);

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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
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
  if (storage.hasShockPropagationParams) {
    if (storage.shockPropagationParams.blockSizes.empty()) {
      AddFindexShockPropagationCounters(
          state, fixture->contactCount, fixture->problem.b.size());
    } else {
      AddShockPropagationCounters(state, storage.shockPropagationParams);
    }
  }
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
          batchKind == WorldContactBatchKind::StressStack
              ? "WorldContactStressBatchSerial/FrictionIndex"
              : "WorldContactBatchSerial/FrictionIndex"));
  if (batchKind == WorldContactBatchKind::StressStack) {
    state.counters["stress_stack_contact_batch"] = 1.0;
    state.counters["separated_contact_shape_count"] = 3.0;
    state.counters["stack_contact_shape_count"] = 4.0;
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
          batchKind == WorldContactBatchKind::StressStack
              ? "WorldContactStressBatchParallel/FrictionIndex"
              : "WorldContactBatchParallel/FrictionIndex"));
  if (batchKind == WorldContactBatchKind::StressStack) {
    state.counters["stress_stack_contact_batch"] = 1.0;
    state.counters["separated_contact_shape_count"] = 3.0;
    state.counters["stack_contact_shape_count"] = 4.0;
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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
      batch->problemGroups, packets, options, iterations);
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL                            \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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

void RunCudaWorldBoxContactBatchBenchmark(benchmark::State& state)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int batchSize = static_cast<int>(state.range(0));
  std::string errorMessage;
  const auto batch
      = MakeHomogeneousWorldBoxContactBenchmarkBatch(batchSize, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  constexpr std::size_t iterations = 1024u;
  auto basePacket = MakeCudaBatchProblem(batch->problems, iterations);
  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packet = basePacket;
    cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
    benchmark::DoNotOptimize(packet.x.data());
  }

  auto packet = basePacket;
  cuda_compute::solveBoxedLcpPgsBatchCuda(packet);
  counters
      = CheckCudaBatchResult(batch->problems, packet.x, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      *batch,
      MakeLabel("PgsCuda", "WorldBoxContactBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_dense_box_contact_batch"] = 1.0;
  state.counters["dense_box_contact"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
  state.counters["contact_count"] = 4.0;
  state.counters["problem_size"] = 12.0;
}

void RunCudaWorldBoxContactGroupedBatchBenchmark(benchmark::State& state)
{
  if (!cuda_compute::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const int variantsPerBoxCount = static_cast<int>(state.range(0));
  constexpr std::size_t iterations = 1024u;
  std::string errorMessage;
  const auto batch = MakeGroupedWorldBoxContactCudaBatch(
      variantsPerBoxCount, iterations, errorMessage);
  if (!batch.has_value()) {
    state.SkipWithError(errorMessage.c_str());
    return;
  }

  const auto options = MakeBenchmarkOptions(static_cast<int>(iterations));
  auto basePackets = batch->packets;

  BatchBenchmarkCounters counters;
  for (auto _ : state) {
    auto packets = basePackets;
    cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
    for (const auto& packet : packets) {
      benchmark::DoNotOptimize(packet.x.data());
    }
  }

  auto packets = basePackets;
  cuda_compute::solveBoxedLcpPgsGroupedBatchCuda(packets);
  counters = CheckCudaGroupedBatchResult(
      batch->problemGroups, packets, options, iterations);
  AddWorldContactBatchCounters(
      state,
      counters,
      batch->aggregate,
      MakeLabel("PgsCuda", "WorldBoxContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_dense_box_contact_batch"] = 1.0;
  state.counters["dense_box_contact"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
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

  const std::size_t iterations = usePgs ? 512u : 1024u;
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
          "WorldStackContactBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_world_stack_contact_batch"] = 1.0;
  state.counters["cuda_coupled_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
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
  const std::size_t iterations = usePgs ? 512u : 1024u;
  std::string errorMessage;
  const auto batch = MakeGroupedWorldStackContactCudaBatch(
      variantsPerSphereCount, iterations, errorMessage);
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
          "WorldStackContactGroupedBatch/FrictionIndex"));
  state.counters["cuda_lcp_execution"] = 1.0;
  state.counters["cuda_batch_execution"] = 1.0;
  state.counters["cuda_grouped_batch_execution"] = 1.0;
  state.counters["cuda_variable_problem_size_batch"] = 1.0;
  state.counters["cuda_world_contact_batch"] = 1.0;
  state.counters["cuda_world_stack_contact_batch"] = 1.0;
  state.counters["cuda_coupled_contact_batch"] = 1.0;
  state.counters["cuda_fixed_iterations"] = static_cast<double>(iterations);
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

void AddBenchmarkArgs(
    benchmark::Benchmark* registeredBenchmark,
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  if (family == BenchmarkProblemFamily::Standard && solver.name == "Direct") {
    // DirectSolver enumerates n <= 3 before falling back to Dantzig.
    registeredBenchmark->Arg(2)->Arg(3);
    return;
  }

  switch (family) {
    case BenchmarkProblemFamily::Standard:
      registeredBenchmark->Arg(12)->Arg(24)->Arg(48)->Arg(96);
      break;
    case BenchmarkProblemFamily::Boxed:
      registeredBenchmark->Arg(12)->Arg(24)->Arg(48);
      break;
    case BenchmarkProblemFamily::FrictionIndex:
      registeredBenchmark->Arg(4)->Arg(16)->Arg(64);
      break;
  }
}

void AddBatchBenchmarkArgs(
    benchmark::Benchmark* registeredBenchmark,
    const dart::test::LcpSolverManifestEntry& solver,
    BenchmarkProblemFamily family)
{
  constexpr int batchSize = 4;
  if (family == BenchmarkProblemFamily::Standard && solver.name == "Direct") {
    // DirectSolver enumerates n <= 3 before falling back to Dantzig.
    registeredBenchmark->Args({3, batchSize});
    return;
  }

  switch (family) {
    case BenchmarkProblemFamily::Standard:
      registeredBenchmark->Args({24, batchSize});
      break;
    case BenchmarkProblemFamily::Boxed:
      registeredBenchmark->Args({24, batchSize});
      break;
    case BenchmarkProblemFamily::FrictionIndex:
      registeredBenchmark->Args({8, batchSize});
      break;
  }
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

std::string MakeNewtonWarmStartBenchmarkName(
    const dart::test::LcpSolverManifestEntry& solver,
    const NewtonWarmStartMode mode)
{
  std::ostringstream out;
  out << "BM_LcpNewtonWarmStart/StandardActiveSet/" << solver.name << "/"
      << getNewtonWarmStartModeName(mode);
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
std::string MakeParallelBatchBenchmarkName(
    BenchmarkProblemFamily family,
    const dart::test::LcpSolverManifestEntry& solver)
{
  std::ostringstream out;
  out << "BM_LcpBatchParallel/" << getProblemFamilyName(family) << "/"
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

  #if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
    const MildIllConditionedBenchmarkCase testCase)
{
  if (!dart::test::supportsProblem(
          solver, getMildIllConditionedProblemSupport(testCase))) {
    return false;
  }

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
    return SolverNameIn(solver, kExactStandardSolvers)
           || SolverNameIn(solver, kScopedSolvers);
  }

  return SolverNameIn(solver, kScopedSolvers);
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

bool SolverShouldRunNearSingularBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const NearSingularBenchmarkCase testCase)
{
  if (!dart::test::supportsProblem(
          solver, getNearSingularProblemSupport(testCase))) {
    return false;
  }

  if (testCase == NearSingularBenchmarkCase::Standard8) {
    constexpr std::array<std::string_view, 2> kStandardSolvers{{
        "Dantzig",
        "Baraff",
    }};
    return SolverNameIn(solver, kStandardSolvers);
  }

  if (testCase == NearSingularBenchmarkCase::Boxed8) {
    constexpr std::array<std::string_view, 3> kBoxedSolvers{{
        "Dantzig",
        "ShockPropagation",
        "BoxedSemiSmoothNewton",
    }};
    return SolverNameIn(solver, kBoxedSolvers);
  }

  constexpr std::array<std::string_view, 2> kFrictionIndexSolvers{{
      "Dantzig",
      "ShockPropagation",
  }};
  return SolverNameIn(solver, kFrictionIndexSolvers);
}
#endif

bool SolverShouldRunLargerActiveSetTransitionBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const LargerActiveSetTransitionBenchmarkCase testCase)
{
  if (!dart::test::supportsProblem(
          solver, getLargerActiveSetTransitionProblemSupport(testCase))) {
    return false;
  }

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

  return SolverNameIn(solver, kScalableSolvers);
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

bool SolverShouldRunSingularDegenerateBenchmark(
    const dart::test::LcpSolverManifestEntry& solver,
    const SingularDegenerateBenchmarkCase testCase)
{
  if (!dart::test::supportsProblem(
          solver, getSingularDegenerateProblemSupport(testCase))) {
    return false;
  }

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
    return SolverNameIn(solver, kStandardSolvers);
  }

  constexpr std::array<std::string_view, 3> kBoxedAndFindexSolvers{{
      "Admm",
      "Sap",
      "BoxedSemiSmoothNewton",
  }};
  return SolverNameIn(solver, kBoxedAndFindexSolvers);
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
      if (!dart::test::supportsProblem(solver, getProblemSupport(family))) {
        continue;
      }

      const auto name = MakeBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestBenchmark(state, solver, family);
          });
      AddBenchmarkArgs(registeredBenchmark, solver, family);
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!dart::test::supportsProblem(solver, getProblemSupport(family))) {
        continue;
      }
      if (family == BenchmarkProblemFamily::Standard
          && solver.name == "Direct") {
        // DirectSolver only enumerates n <= 3. The active-set transition
        // standard packet is 16-row, so skip the Dantzig fallback.
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

    for (const auto mode : kModes) {
      const auto name = MakeNewtonWarmStartBenchmarkName(solver, mode);
      benchmark::RegisterBenchmark(
          name.c_str(),
          [solver, mode](benchmark::State& state) {
            RunNewtonWarmStartBenchmark(state, solver, mode);
          })
          ->Arg(32)
          ->Arg(64);

      const auto serialBatchName
          = MakeNewtonWarmStartBatchSerialBenchmarkName(solver, mode);
      benchmark::RegisterBenchmark(
          serialBatchName.c_str(),
          [solver, mode](benchmark::State& state) {
            RunNewtonWarmStartBatchSerialBenchmark(state, solver, mode);
          })
          ->Args({32, 4})
          ->Args({64, 4});

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
      const auto parallelBatchName
          = MakeNewtonWarmStartBatchParallelBenchmarkName(solver, mode);
      benchmark::RegisterBenchmark(
          parallelBatchName.c_str(),
          [solver, mode](benchmark::State& state) {
            RunNewtonWarmStartBatchParallelBenchmark(state, solver, mode);
          })
          ->Args({32, 4})
          ->Args({64, 4});
#endif
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(
              solver, testCase)) {
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(
              solver, testCase)) {
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(
              solver, testCase)) {
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
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 2> cases{
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32};

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(
              solver, testCase)) {
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
  constexpr std::array<LargerActiveSetTransitionBenchmarkCase, 2> cases{
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex24,
      LargerActiveSetTransitionBenchmarkCase::CoupledFrictionIndex32};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunLargerActiveSetTransitionBenchmark(
              solver, testCase)) {
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  constexpr std::array<MildIllConditionedBenchmarkCase, 12> cases{
      MildIllConditionedBenchmarkCase::Standard32,
      MildIllConditionedBenchmarkCase::Boxed16,
      MildIllConditionedBenchmarkCase::FrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex6,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex8,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex12,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::CoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongCoupledFrictionIndex24,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24};

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunMildIllConditionedBenchmark(solver, testCase)) {
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
  constexpr std::array<MildIllConditionedBenchmarkCase, 2> cases{
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex16,
      MildIllConditionedBenchmarkCase::StrongerCoupledFrictionIndex24};
  constexpr int batchSize = 4;

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunMildIllConditionedBenchmark(solver, testCase)) {
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
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
  constexpr std::array<NearSingularBenchmarkCase, 6> cases{
      NearSingularBenchmarkCase::Standard8,
      NearSingularBenchmarkCase::Boxed8,
      NearSingularBenchmarkCase::CoupledFrictionIndex3,
      NearSingularBenchmarkCase::CoupledFrictionIndex6,
      NearSingularBenchmarkCase::CoupledFrictionIndex9,
      NearSingularBenchmarkCase::CoupledFrictionIndex12};

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunNearSingularBenchmark(solver, testCase)) {
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

void RegisterSingularDegenerateBenchmarks()
{
  constexpr std::array<SingularDegenerateBenchmarkCase, 3> cases{
      SingularDegenerateBenchmarkCase::Standard16,
      SingularDegenerateBenchmarkCase::Boxed16,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex6};

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(solver, testCase)) {
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(solver, testCase)) {
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
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(solver, testCase)) {
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
  constexpr std::array<SingularDegenerateBenchmarkCase, 3> cases{
      SingularDegenerateBenchmarkCase::Standard128,
      SingularDegenerateBenchmarkCase::Boxed128,
      SingularDegenerateBenchmarkCase::CoupledFrictionIndex16};

  for (const auto testCase : cases) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!SolverShouldRunSingularDegenerateBenchmark(solver, testCase)) {
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

void RegisterBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!dart::test::supportsProblem(solver, getProblemSupport(family))) {
        continue;
      }

      const auto name = MakeBatchBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestBatchBenchmark(state, solver, family);
          });
      AddBatchBenchmarkArgs(registeredBenchmark, solver, family);
    }
  }
}

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
void RegisterParallelBatchBenchmarks()
{
  constexpr std::array<BenchmarkProblemFamily, 3> families{
      BenchmarkProblemFamily::Standard,
      BenchmarkProblemFamily::Boxed,
      BenchmarkProblemFamily::FrictionIndex};

  for (const auto family : families) {
    for (const auto& solver : dart::test::kLcpSolverManifest) {
      if (!dart::test::supportsProblem(solver, getProblemSupport(family))) {
        continue;
      }

      const auto name = MakeParallelBatchBenchmarkName(family, solver);
      auto* registeredBenchmark = benchmark::RegisterBenchmark(
          name.c_str(), [solver, family](benchmark::State& state) {
            RunManifestParallelBatchBenchmark(state, solver, family);
          });
      AddBatchBenchmarkArgs(registeredBenchmark, solver, family);
    }
  }
}

void RegisterWorldContactBenchmarks()
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!dart::test::supportsProblem(
            solver, dart::test::LcpProblemSupport::FrictionIndex)) {
      continue;
    }

    const auto name = MakeWorldContactBenchmarkName(solver);
    benchmark::RegisterBenchmark(
        name.c_str(),
        [solver](benchmark::State& state) {
          RunWorldContactBenchmark(state, solver);
        })
        ->Arg(1)
        ->Arg(2)
        ->Arg(4);
  }
}

void RegisterWorldBoxContactBenchmarks()
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!dart::test::supportsProblem(
            solver, dart::test::LcpProblemSupport::FrictionIndex)) {
      continue;
    }
    if (!SupportsDenseWorldBoxContactPatch(solver.name)) {
      continue;
    }

    const auto name = MakeWorldBoxContactBenchmarkName(solver);
    benchmark::RegisterBenchmark(
        name.c_str(),
        [solver](benchmark::State& state) {
          RunWorldBoxContactBenchmark(state, solver);
        })
        ->Arg(1)
        ->Arg(2)
        ->Arg(4);
  }
}

void RegisterWorldStackContactBenchmarks()
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!dart::test::supportsProblem(
            solver, dart::test::LcpProblemSupport::FrictionIndex)) {
      continue;
    }

    const auto name = MakeWorldStackContactBenchmarkName(solver);
    auto* registeredBenchmark = benchmark::RegisterBenchmark(
        name.c_str(), [solver](benchmark::State& state) {
          RunWorldStackContactBenchmark(state, solver);
        });
    registeredBenchmark->Arg(2)->Arg(3);
    if (solver.name != "NNCG") {
      registeredBenchmark->Arg(4)->Arg(5);
    }
  }
}

void RegisterArticulatedUnifiedContactBenchmarks()
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!dart::test::supportsProblem(
            solver, dart::test::LcpProblemSupport::FrictionIndex)) {
      continue;
    }

    for (const auto benchmarkCase : std::array{
             ArticulatedContactBenchmarkCase::Ground,
             ArticulatedContactBenchmarkCase::RigidImpact,
             ArticulatedContactBenchmarkCase::CrossLinkImpact}) {
      const auto name
          = MakeArticulatedUnifiedContactBenchmarkName(benchmarkCase, solver);
      benchmark::RegisterBenchmark(
          name.c_str(),
          [solver, benchmarkCase](benchmark::State& state) {
            RunArticulatedUnifiedContactBenchmark(state, solver, benchmarkCase);
          })
          ->Arg(1)
          ->Arg(4);
    }
  }
}

void RegisterWorldContactBatchBenchmarks()
{
  for (const auto& solver : dart::test::kLcpSolverManifest) {
    if (!dart::test::supportsProblem(
            solver, dart::test::LcpProblemSupport::FrictionIndex)) {
      continue;
    }

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

    if (solver.name != "NNCG") {
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL                            \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
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

static void BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactBatchBenchmark(state);
}

static void BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex(
    benchmark::State& state)
{
  RunCudaWorldBoxContactGroupedBatchBenchmark(state);
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
  RegisterNewtonWarmStartBenchmarks();
  RegisterLargerActiveSetTransitionBenchmarks();
  RegisterStressActiveSetTransitionBenchmarks();
  RegisterExtremeActiveSetTransitionBenchmarks();
  RegisterProductionActiveSetTransitionBenchmarks();
  RegisterProductionActiveSetTransitionBatchBenchmarks();
  RegisterMildIllConditionedBenchmarks();
  RegisterMildIllConditionedBatchBenchmarks();
  RegisterNearSingularBenchmarks();
  RegisterSingularDegenerateBenchmarks();
  RegisterLargerSingularDegenerateBenchmarks();
  RegisterStressSingularDegenerateBenchmarks();
  RegisterExtremeSingularDegenerateBenchmarks();
  RegisterBatchBenchmarks();
#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
  RegisterParallelBatchBenchmarks();
  RegisterWorldContactBenchmarks();
  RegisterWorldBoxContactBenchmarks();
  RegisterWorldStackContactBenchmarks();
  RegisterArticulatedUnifiedContactBenchmarks();
  RegisterWorldContactBatchBenchmarks();
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

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
BENCHMARK(BM_LcpCudaJacobiBatch_Standard)->Args({24, 4})->Args({48, 4});
BENCHMARK(BM_LcpCudaJacobiBatch_Boxed)->Args({24, 4})->Args({48, 4});
BENCHMARK(BM_LcpCudaJacobiBatch_FrictionIndex)->Args({8, 4})->Args({16, 4});
BENCHMARK(BM_LcpCudaPgsBatch_Standard)->Args({24, 4})->Args({48, 4});
BENCHMARK(BM_LcpCudaPgsBatch_Boxed)->Args({24, 4})->Args({48, 4});
BENCHMARK(BM_LcpCudaPgsBatch_FrictionIndex)->Args({8, 4})->Args({16, 4});
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_Standard)->Arg(2);
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_Boxed)->Arg(2);
BENCHMARK(BM_LcpCudaJacobiGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_Standard)->Arg(2);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_Boxed)->Arg(2);
BENCHMARK(BM_LcpCudaPgsGroupedBatch_FrictionIndex)->Arg(2);
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL                            \
    && DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL_CUDA
BENCHMARK(BM_LcpCudaJacobiWorldContactBatch_FrictionIndex)
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4});
BENCHMARK(BM_LcpCudaPgsWorldContactBatch_FrictionIndex)
    ->Args({4, 4})
    ->Args({8, 4})
    ->Args({16, 4});
BENCHMARK(BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex)->Arg(4);
BENCHMARK(BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaJacobiWorldStackContactBatch_FrictionIndex)->Args({5, 4});
BENCHMARK(BM_LcpCudaPgsWorldStackContactBatch_FrictionIndex)->Args({5, 4});
BENCHMARK(BM_LcpCudaJacobiWorldContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaPgsWorldContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaJacobiWorldStackContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaPgsWorldStackContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch_FrictionIndex)
    ->Arg(2);
BENCHMARK(BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch_FrictionIndex)
    ->Arg(2);
BENCHMARK(BM_LcpCudaJacobiMixedContactGroupedBatch_FrictionIndex)->Arg(2);
BENCHMARK(BM_LcpCudaPgsMixedContactGroupedBatch_FrictionIndex)->Arg(2);
#endif

#if DART_BM_LCP_COMPARE_HAS_SIMULATION_EXPERIMENTAL
BENCHMARK(BM_LcpWorldContactAssembly_BoxedLcp)->Arg(1)->Arg(2)->Arg(4);
BENCHMARK(BM_LcpWorldStackContactAssembly_BoxedLcp)
    ->Arg(2)
    ->Arg(3)
    ->Arg(4)
    ->Arg(5);
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({3, 200})->Args({3, 500});
BENCHMARK(BM_LcpWorldStackStep_BoxedLcp)->Args({4, 200});
BENCHMARK(BM_LcpWorldSeparatedStep_BoxedLcp)
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200});
BENCHMARK(BM_LcpWorldBoxStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({2, 200})
    ->Args({4, 200});
BENCHMARK(BM_LcpWorldArticulatedGroundStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200});
BENCHMARK(BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp)
    ->Args({1, 1})
    ->Args({4, 1})
    ->Args({8, 1})
    ->Args({16, 1});
BENCHMARK(BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp)
    ->Args({1, 1})
    ->Args({4, 1})
    ->Args({8, 1})
    ->Args({16, 1});
BENCHMARK(BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp)
    ->Args({1, 200})
    ->Args({4, 200})
    ->Args({8, 200})
    ->Args({16, 200});
#endif

BENCHMARK(BM_LCP_COMPARE_SMOKE);
