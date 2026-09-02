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

#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/neo_hookean.hpp>
#include <dart/simulation/detail/deformable_vbd/parallel_block_descent.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <stdexcept>
#include <vector>

#include <cmath>
#include <cstdint>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace compute = dart::simulation::compute;

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;

struct PhysicalLameParameters
{
  double mu;
  double lambda;
};

//==============================================================================
// Independent engineering conversion used only as the Hooke-law oracle. The
// production conversion intentionally returns Smith model coefficients.
PhysicalLameParameters physicalLame(double youngsModulus, double poissonRatio)
{
  return {
      youngsModulus / (2.0 * (1.0 + poissonRatio)),
      youngsModulus * poissonRatio
          / ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio))};
}

//==============================================================================
std::array<Eigen::Matrix3d, 6> symmetricStrainBasis()
{
  std::array<Eigen::Matrix3d, 6> basis;
  for (auto& direction : basis) {
    direction.setZero();
  }
  basis[0](0, 0) = 1.0;
  basis[1](1, 1) = 1.0;
  basis[2](2, 2) = 1.0;
  basis[3](0, 1) = basis[3](1, 0) = 0.5;
  basis[4](0, 2) = basis[4](2, 0) = 0.5;
  basis[5](1, 2) = basis[5](2, 1) = 0.5;
  return basis;
}

//==============================================================================
double hookeTangent(
    const PhysicalLameParameters& physical,
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b)
{
  return 2.0 * physical.mu * (a.array() * b.array()).sum()
         + physical.lambda * a.trace() * b.trace();
}

//==============================================================================
double finiteEnergyHessian(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b,
    double muHat,
    double lambdaHat)
{
  // The rest-zeroed energy subtracts nearly equal O(h) terms. A moderately
  // larger step keeps exactly zero Hooke cross terms above roundoff while
  // remaining well inside the quadratic small-strain regime.
  constexpr double h = 1e-4;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const auto energy = [&](const Eigen::Matrix3d& direction) {
    return vbd::stableNeoHookeanEnergyDensity(
        identity + h * direction, muHat, lambdaHat);
  };
  return (energy(a + b) - energy(a - b) - energy(-a + b) + energy(-a - b))
         / (4.0 * h * h);
}

//==============================================================================
double finiteStressTangent(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b,
    double muHat,
    double lambdaHat)
{
  constexpr double h = 1e-6;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d tangent
      = (vbd::stableNeoHookeanStress(identity + h * b, muHat, lambdaHat)
         - vbd::stableNeoHookeanStress(identity - h * b, muHat, lambdaHat))
        / (2.0 * h);
  return (a.array() * tangent.array()).sum();
}

//==============================================================================
// A reference unit tetrahedron (positive volume).
Tet referenceTet()
{
  return {
      Vec3(0.0, 0.0, 0.0),
      Vec3(1.0, 0.0, 0.0),
      Vec3(0.0, 1.0, 0.0),
      Vec3(0.0, 0.0, 1.0)};
}

//==============================================================================
// Total tet energy A * Psi(F) as a function of one vertex's position, used as
// the finite-difference oracle.
double tetEnergy(
    const vbd::TetRestShape& rest,
    const Tet& positions,
    double mu,
    double lambda)
{
  const Eigen::Matrix3d F
      = vbd::deformationGradient(rest.restShapeInverse, positions);
  return rest.restVolume * vbd::stableNeoHookeanEnergyDensity(F, mu, lambda);
}

//==============================================================================
Vec3 numericForce(
    const vbd::TetRestShape& rest,
    Tet positions,
    int vertex,
    double mu,
    double lambda,
    double eps = 1e-6)
{
  Vec3 grad = Vec3::Zero();
  for (int d = 0; d < 3; ++d) {
    Tet plus = positions;
    Tet minus = positions;
    plus[vertex][d] += eps;
    minus[vertex][d] -= eps;
    const double ep = tetEnergy(rest, plus, mu, lambda);
    const double em = tetEnergy(rest, minus, mu, lambda);
    grad[d] = (ep - em) / (2.0 * eps);
  }
  return -grad; // force = -dE/dx
}

//==============================================================================
Eigen::Matrix3d numericHessian(
    const vbd::TetRestShape& rest,
    Tet positions,
    int vertex,
    double mu,
    double lambda,
    double eps = 1e-4)
{
  Eigen::Matrix3d hess = Eigen::Matrix3d::Zero();
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      Tet pp = positions;
      Tet pm = positions;
      Tet mp = positions;
      Tet mm = positions;
      pp[vertex][a] += eps;
      pp[vertex][b] += eps;
      pm[vertex][a] += eps;
      pm[vertex][b] -= eps;
      mp[vertex][a] -= eps;
      mp[vertex][b] += eps;
      mm[vertex][a] -= eps;
      mm[vertex][b] -= eps;
      hess(a, b)
          = (tetEnergy(rest, pp, mu, lambda) - tetEnergy(rest, pm, mu, lambda)
             - tetEnergy(rest, mp, mu, lambda)
             + tetEnergy(rest, mm, mu, lambda))
            / (4.0 * eps * eps);
    }
  }
  return hess;
}

} // namespace

//==============================================================================
TEST(VbdNeoHookean, SmithCoefficientConversionMatchesFormulas)
{
  constexpr double youngsModulus = 1.0e5;
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const PhysicalLameParameters physical
        = physicalLame(youngsModulus, poissonRatio);
    const vbd::LameParameters model
        = vbd::lameFromYoungPoisson(youngsModulus, poissonRatio);
    EXPECT_NEAR(model.mu, physical.mu, 1e-12 * physical.mu);
    EXPECT_NEAR(
        model.lambda,
        physical.lambda + physical.mu,
        1e-12 * (1.0 + std::abs(model.lambda)));
  }
}

//==============================================================================
TEST(VbdNeoHookean, AuxeticEngineeringMaterialsAreRejected)
{
  for (const double poissonRatio : {-0.9, -0.5, -1e-12}) {
    SCOPED_TRACE(poissonRatio);
    EXPECT_FALSE(vbd::isSupportedPoissonRatio(poissonRatio));
    EXPECT_THROW(
        static_cast<void>(vbd::lameFromYoungPoisson(1.0e5, poissonRatio)),
        std::invalid_argument);
  }
  for (const double poissonRatio :
       {0.5, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(poissonRatio);
    EXPECT_FALSE(vbd::isSupportedPoissonRatio(poissonRatio));
    EXPECT_THROW(
        static_cast<void>(vbd::lameFromYoungPoisson(1.0e5, poissonRatio)),
        std::invalid_argument);
  }
}

//==============================================================================
// The rejected auxetic mapping is locally Hookean but not globally rest-stable.
// At complete uniform collapse, the no-log Smith energy is exactly
// (lambdaHat-muHat)/2 = lambda_L/2, which is negative for every nu<0.
TEST(VbdNeoHookean, AuxeticMappingWouldCreateCollapsedLowerEnergyWell)
{
  constexpr double youngsModulus = 6000.0;
  constexpr double poissonRatio = -0.5;
  const PhysicalLameParameters physical
      = physicalLame(youngsModulus, poissonRatio);
  const double muHat = physical.mu;
  const double lambdaHat = physical.lambda + physical.mu;
  const Eigen::Matrix3d rest = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d collapsed = Eigen::Matrix3d::Zero();
  EXPECT_DOUBLE_EQ(
      vbd::stableNeoHookeanEnergyDensity(rest, muHat, lambdaHat), 0.0);
  EXPECT_NEAR(
      vbd::stableNeoHookeanEnergyDensity(collapsed, muHat, lambdaHat),
      0.5 * physical.lambda,
      1e-12 * std::abs(physical.lambda));
  EXPECT_LT(
      vbd::stableNeoHookeanEnergyDensity(collapsed, muHat, lambdaHat), 0.0);
}

//==============================================================================
// At the admitted nu=0 boundary, uniform collapse is an equal-energy state but
// no lower well exists. Positive nu makes rest the unique sampled minimum.
TEST(VbdNeoHookean, SupportedUniformScaleEnergyLandscapeIsNonnegative)
{
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const vbd::LameParameters model
        = vbd::lameFromYoungPoisson(6000.0, poissonRatio);
    for (const double scale : {0.0, 0.2, 0.6, 1.0, 1.4, 2.0}) {
      SCOPED_TRACE(scale);
      const double energy = vbd::stableNeoHookeanEnergyDensity(
          scale * Eigen::Matrix3d::Identity(), model.mu, model.lambda);
      EXPECT_GE(energy, -1e-12 * model.mu);
      if (poissonRatio > 0.0 && scale != 1.0) {
        EXPECT_GT(energy, 0.0);
      }
    }
  }
}

//==============================================================================
// This is an independent material-calibration test: the expected tangent comes
// directly from engineering Hooke elasticity, while the two actual values are
// numerical derivatives of the nonlinear Smith energy and stress. It catches
// a self-consistent but physically mis-parameterized energy/gradient/Hessian.
TEST(VbdNeoHookean, SmallStrainEnergyHessianAndTangentMatchHookeLaw)
{
  constexpr double youngsModulus = 12345.0;
  const auto basis = symmetricStrainBasis();
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const PhysicalLameParameters physical
        = physicalLame(youngsModulus, poissonRatio);
    const vbd::LameParameters model
        = vbd::lameFromYoungPoisson(youngsModulus, poissonRatio);
    for (std::size_t row = 0; row < basis.size(); ++row) {
      for (std::size_t col = 0; col < basis.size(); ++col) {
        SCOPED_TRACE(::testing::Message() << "row=" << row << " col=" << col);
        const double expected = hookeTangent(physical, basis[row], basis[col]);
        const double energyHessian = finiteEnergyHessian(
            basis[row], basis[col], model.mu, model.lambda);
        const double stressTangent = finiteStressTangent(
            basis[row], basis[col], model.mu, model.lambda);
        const double energyTolerance = 5e-5 + 2e-5 * std::abs(expected);
        const double stressTolerance = 2e-5 * (1.0 + std::abs(expected));
        EXPECT_NEAR(energyHessian, expected, energyTolerance);
        EXPECT_NEAR(stressTangent, expected, stressTolerance);
      }
    }
  }
}

//==============================================================================
TEST(VbdNeoHookean, EngineeringZeroPoissonHasFiniteDifferentiableLimit)
{
  const vbd::LameParameters zero = vbd::lameFromYoungPoisson(6000.0, 0.0);
  const vbd::LameParameters nearZero = vbd::lameFromYoungPoisson(6000.0, 1e-12);
  ASSERT_DOUBLE_EQ(zero.mu, 3000.0);
  ASSERT_DOUBLE_EQ(zero.lambda, zero.mu);

  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);
  Tet deformed = rest;
  deformed[1] += Vec3(0.20, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.18, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.22);
  const Eigen::Matrix3d F
      = vbd::deformationGradient(shape.restShapeInverse, deformed);

  const double energy
      = vbd::stableNeoHookeanEnergyDensity(F, zero.mu, zero.lambda);
  const double nearZeroEnergy
      = vbd::stableNeoHookeanEnergyDensity(F, nearZero.mu, nearZero.lambda);
  EXPECT_TRUE(std::isfinite(energy));
  EXPECT_NEAR(energy, nearZeroEnergy, 1e-8 * (1.0 + std::abs(energy)));

  const Eigen::Matrix3d stress
      = vbd::stableNeoHookeanStress(F, zero.mu, zero.lambda);
  const Eigen::Matrix3d nearZeroStress
      = vbd::stableNeoHookeanStress(F, nearZero.mu, nearZero.lambda);
  EXPECT_TRUE(stress.allFinite());
  EXPECT_NEAR(
      (stress - nearZeroStress).norm(), 0.0, 1e-8 * (1.0 + stress.norm()));

  for (int vertex = 0; vertex < 4; ++vertex) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(
        block, vertex, shape, deformed, zero.mu, zero.lambda);
    vbd::VertexBlock nearZeroBlock;
    vbd::addNeoHookeanTetTerm(
        nearZeroBlock, vertex, shape, deformed, nearZero.mu, nearZero.lambda);
    EXPECT_TRUE(block.force.allFinite()) << "vertex " << vertex;
    EXPECT_TRUE(block.hessian.allFinite()) << "vertex " << vertex;
    EXPECT_NEAR(
        (block.force - nearZeroBlock.force).norm(),
        0.0,
        1e-8 * (1.0 + block.force.norm()))
        << "vertex " << vertex;
    EXPECT_NEAR(
        (block.hessian - nearZeroBlock.hessian).norm(),
        0.0,
        1e-8 * (1.0 + block.hessian.norm()))
        << "vertex " << vertex;

    const Vec3 finiteDifferenceForce
        = numericForce(shape, deformed, vertex, zero.mu, zero.lambda);
    EXPECT_NEAR(
        (block.force - finiteDifferenceForce).norm(),
        0.0,
        1e-2 * (1.0 + finiteDifferenceForce.norm()))
        << "vertex " << vertex;
    const Eigen::Matrix3d finiteDifferenceHessian
        = numericHessian(shape, deformed, vertex, zero.mu, zero.lambda);
    EXPECT_NEAR(
        (block.hessian - finiteDifferenceHessian).norm(),
        0.0,
        1e-3 * (1.0 + finiteDifferenceHessian.norm()))
        << "vertex " << vertex;
  }
}

//==============================================================================
TEST(VbdNeoHookean, RawZeroModelLambdaHasFiniteDifferentiableContinuation)
{
  // A raw lambdaHat of zero is not engineering nu=0, but the low-level Smith
  // kernel explicitly supports this algebraic safety edge.
  constexpr double muHat = 3000.0;
  constexpr double lambdaHat = 0.0;
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);
  Tet deformed = rest;
  deformed[1] += Vec3(0.20, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.18, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.22);
  const Eigen::Matrix3d f
      = vbd::deformationGradient(shape.restShapeInverse, deformed);
  EXPECT_TRUE(
      std::isfinite(vbd::stableNeoHookeanEnergyDensity(f, muHat, lambdaHat)));
  EXPECT_TRUE(vbd::stableNeoHookeanStress(f, muHat, lambdaHat).allFinite());

  for (int vertex = 0; vertex < 4; ++vertex) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, vertex, shape, deformed, muHat, lambdaHat);
    EXPECT_TRUE(block.force.allFinite()) << "vertex " << vertex;
    EXPECT_TRUE(block.hessian.allFinite()) << "vertex " << vertex;
    const Vec3 finiteDifferenceForce
        = numericForce(shape, deformed, vertex, muHat, lambdaHat);
    EXPECT_NEAR(
        (block.force - finiteDifferenceForce).norm(),
        0.0,
        1e-2 * (1.0 + finiteDifferenceForce.norm()))
        << "vertex " << vertex;
    const Eigen::Matrix3d finiteDifferenceHessian
        = numericHessian(shape, deformed, vertex, muHat, lambdaHat);
    EXPECT_NEAR(
        (block.hessian - finiteDifferenceHessian).norm(),
        0.0,
        1e-3 * (1.0 + finiteDifferenceHessian.norm()))
        << "vertex " << vertex;
  }
}

//==============================================================================
TEST(VbdNeoHookean, FiniteExtremeEngineeringInputsDoNotOverflowKernel)
{
  const double poisson = std::numeric_limits<double>::denorm_min();
  const vbd::LameParameters lame = vbd::lameFromYoungPoisson(1e100, poisson);
  ASSERT_TRUE(std::isfinite(lame.mu));
  ASSERT_TRUE(std::isfinite(lame.lambda));
  ASSERT_GT(lame.mu, 0.0);
  ASSERT_GT(lame.lambda, 0.0);
  ASSERT_TRUE(std::isfinite(lame.mu / lame.lambda));
  EXPECT_DOUBLE_EQ(lame.lambda, lame.mu);

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 0) = 1.1;
  F(1, 1) = 0.9;
  F(0, 1) = 0.05;
  const double energy
      = vbd::stableNeoHookeanEnergyDensity(F, lame.mu, lame.lambda);
  const Eigen::Matrix3d stress
      = vbd::stableNeoHookeanStress(F, lame.mu, lame.lambda);
  EXPECT_TRUE(std::isfinite(energy));
  EXPECT_TRUE(stress.allFinite());

  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);
  Tet deformed = rest;
  deformed[1] = F.col(0);
  deformed[2] = F.col(1);
  deformed[3] = F.col(2);
  for (int vertex = 0; vertex < 4; ++vertex) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(
        block, vertex, shape, deformed, lame.mu, lame.lambda);
    EXPECT_TRUE(block.force.allFinite()) << "vertex " << vertex;
    EXPECT_TRUE(block.hessian.allFinite()) << "vertex " << vertex;
  }
}

//==============================================================================
TEST(VbdNeoHookean, EngineeringZeroPoissonTetSolveMatchesSequentialAndParallel)
{
  const std::vector<Vec3> rest{
      Vec3(0.0, 0.0, 0.0),
      Vec3(1.0, 0.0, 0.0),
      Vec3(0.0, 1.0, 0.0),
      Vec3(0.0, 0.0, 1.0),
      Vec3(3.0, 0.0, 0.0),
      Vec3(4.0, 0.0, 0.0),
      Vec3(3.0, 1.0, 0.0),
      Vec3(3.0, 0.0, 1.0)};
  const std::vector<vbd::SpringElement> springs;
  const auto tetRest = [&](const std::array<std::uint32_t, 4>& vertices) {
    return vbd::makeTetRestShape(
        {rest[vertices[0]],
         rest[vertices[1]],
         rest[vertices[2]],
         rest[vertices[3]]});
  };
  const std::vector<vbd::TetMeshElement> tets{
      {{0u, 1u, 2u, 3u}, tetRest({0u, 1u, 2u, 3u})},
      {{4u, 5u, 6u, 7u}, tetRest({4u, 5u, 6u, 7u})}};
  const std::vector<double> masses(rest.size(), 1.0);
  std::vector<std::uint8_t> fixed(rest.size(), 0u);
  fixed[0] = 1u;
  fixed[4] = 1u;
  std::vector<Vec3> inertialTargets = rest;
  for (std::size_t vertex = 0; vertex < rest.size(); ++vertex) {
    if (fixed[vertex] == 0u) {
      inertialTargets[vertex]
          += Vec3(0.01 * static_cast<double>(vertex + 1u), -0.02, 0.015);
    }
  }
  const auto springAdjacency
      = vbd::SpringAdjacency::build(rest.size(), springs);
  const auto tetAdjacency = vbd::TetAdjacency::build(rest.size(), tets);
  const auto coloring = vbd::colorDeformable(rest.size(), springs, tets);
  const vbd::LameParameters lame = vbd::lameFromYoungPoisson(1000.0, 0.0);
  vbd::BlockDescentOptions options;
  options.iterations = 20u;

  std::vector<Vec3> sequential = inertialTargets;
  const vbd::BlockDescentStats sequentialStats = vbd::blockDescentDeformable(
      sequential,
      masses,
      fixed,
      inertialTargets,
      springs,
      /*springStiffness=*/0.0,
      springAdjacency,
      tets,
      lame.mu,
      lame.lambda,
      tetAdjacency,
      /*timeStep=*/0.01,
      coloring,
      options);
  EXPECT_TRUE(std::isfinite(sequentialStats.finalResidualNormSquared));

  std::vector<Vec3> parallel = inertialTargets;
  compute::ParallelExecutor executor(4u);
  const vbd::BlockDescentStats parallelStats
      = vbd::parallelBlockDescentDeformable(
          parallel,
          masses,
          fixed,
          inertialTargets,
          springs,
          /*springStiffness=*/0.0,
          springAdjacency,
          tets,
          lame.mu,
          lame.lambda,
          tetAdjacency,
          /*timeStep=*/0.01,
          coloring,
          options,
          executor);
  EXPECT_TRUE(std::isfinite(parallelStats.finalResidualNormSquared));
  ASSERT_EQ(sequential.size(), parallel.size());
  for (std::size_t vertex = 0; vertex < sequential.size(); ++vertex) {
    EXPECT_TRUE(sequential[vertex].allFinite()) << "vertex " << vertex;
    EXPECT_TRUE(parallel[vertex].allFinite()) << "vertex " << vertex;
    EXPECT_NEAR((sequential[vertex] - parallel[vertex]).norm(), 0.0, 1e-12)
        << "vertex " << vertex;
  }
}

//==============================================================================
TEST(VbdNeoHookean, RestStateHasZeroEnergyAndStress)
{
  // F = I at rest -> P = muHat I - muHat I = 0, including a raw model
  // lambdaHat of zero.
  const Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  EXPECT_DOUBLE_EQ(vbd::stableNeoHookeanEnergyDensity(F, 4000.0, 8000.0), 0.0);
  EXPECT_DOUBLE_EQ(vbd::stableNeoHookeanEnergyDensity(F, 4000.0, 0.0), 0.0);
  const Eigen::Matrix3d stress = vbd::stableNeoHookeanStress(F, 4000.0, 8000.0);
  EXPECT_NEAR(stress.norm(), 0.0, 1e-9);
}

//==============================================================================
TEST(VbdNeoHookean, RestStateHasZeroForceOnEveryVertex)
{
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);
  for (int v = 0; v < 4; ++v) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, v, shape, rest, 4000.0, 8000.0);
    EXPECT_NEAR(block.force.norm(), 0.0, 1e-7) << "vertex " << v;
  }
}

//==============================================================================
TEST(VbdNeoHookean, ForceMatchesFiniteDifference)
{
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);

  // A stretched/sheared configuration (positive volume).
  Tet deformed = rest;
  deformed[1] += Vec3(0.30, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.22, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.28);

  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const vbd::LameParameters model
        = vbd::lameFromYoungPoisson(6000.0, poissonRatio);
    for (int v = 0; v < 4; ++v) {
      vbd::VertexBlock block;
      vbd::addNeoHookeanTetTerm(
          block, v, shape, deformed, model.mu, model.lambda);
      const Vec3 numeric
          = numericForce(shape, deformed, v, model.mu, model.lambda);
      EXPECT_NEAR(
          (block.force - numeric).norm(), 0.0, 1e-2 * (1.0 + numeric.norm()))
          << "vertex " << v;
    }
  }
}

//==============================================================================
TEST(VbdNeoHookean, HessianMatchesFiniteDifference)
{
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);

  Tet deformed = rest;
  deformed[1] += Vec3(0.20, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.18, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.22);

  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const vbd::LameParameters model
        = vbd::lameFromYoungPoisson(6000.0, poissonRatio);
    for (int v = 0; v < 4; ++v) {
      vbd::VertexBlock block;
      vbd::addNeoHookeanTetTerm(
          block, v, shape, deformed, model.mu, model.lambda);
      const Eigen::Matrix3d numeric
          = numericHessian(shape, deformed, v, model.mu, model.lambda);
      EXPECT_NEAR((block.hessian - numeric).norm(), 0.0, 1.0)
          << "vertex " << v << "\nanalytic=\n"
          << block.hessian << "\nnumeric=\n"
          << numeric;
    }
  }
}

//==============================================================================
TEST(VbdNeoHookean, FiniteAndStableUnderInversion)
{
  const double mu = 3000.0;
  const double lambda = 6000.0;
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);

  // Push vertex 3 through the opposite face so det(F) < 0 (inverted element).
  Tet inverted = rest;
  inverted[3] = Vec3(0.0, 0.0, -0.5);

  const Eigen::Matrix3d F
      = vbd::deformationGradient(shape.restShapeInverse, inverted);
  EXPECT_LT(F.determinant(), 0.0);

  const double energy = vbd::stableNeoHookeanEnergyDensity(F, mu, lambda);
  EXPECT_TRUE(std::isfinite(energy));

  for (int v = 0; v < 4; ++v) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, v, shape, inverted, mu, lambda);
    EXPECT_TRUE(block.force.allFinite()) << "vertex " << v;
    EXPECT_TRUE(block.hessian.allFinite()) << "vertex " << v;

    // The force still matches finite differences even under inversion, since
    // the stable energy has no log term.
    const Vec3 numeric = numericForce(shape, inverted, v, mu, lambda);
    EXPECT_NEAR(
        (block.force - numeric).norm(), 0.0, 1e-2 * (1.0 + numeric.norm()))
        << "vertex " << v;
  }
}

//==============================================================================
TEST(VbdNeoHookean, InertiaAnchoredBlockIsPositiveDefinite)
{
  // Even when the bare element Hessian is indefinite (inverted element), adding
  // the inertia term yields a positive-definite per-vertex block for a solve.
  const double mu = 3000.0;
  const double lambda = 6000.0;
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);
  Tet inverted = rest;
  inverted[3] = Vec3(0.0, 0.0, -0.5);

  vbd::VertexBlock block;
  vbd::addInertiaTerm(
      block, /*mass=*/1.0, /*timeStep=*/1e-3, inverted[3], rest[3]);
  vbd::addNeoHookeanTetTerm(block, 3, shape, inverted, mu, lambda);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(block.hessian);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);

  // The block solve then yields a finite, nonzero descent step.
  const Vec3 delta = vbd::solveVertexBlock(block);
  EXPECT_TRUE(delta.allFinite());
}
