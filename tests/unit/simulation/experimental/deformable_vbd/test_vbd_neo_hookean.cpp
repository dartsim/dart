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

#include <dart/simulation/experimental/detail/deformable_vbd/neo_hookean.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <array>

#include <cmath>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;

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
TEST(VbdNeoHookean, LameConversionMatchesFormulas)
{
  const vbd::LameParameters lame = vbd::lameFromYoungPoisson(1.0e5, 0.3);
  EXPECT_NEAR(lame.mu, 1.0e5 / (2.0 * 1.3), 1e-3);
  EXPECT_NEAR(lame.lambda, 1.0e5 * 0.3 / (1.3 * 0.4), 1e-3);
}

//==============================================================================
TEST(VbdNeoHookean, RestStateHasZeroStress)
{
  // F = I at rest -> P = mu I + lambda (1 - a) I = 0 since a = 1 + mu/lambda.
  const Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
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
  const double mu = 3000.0;
  const double lambda = 6000.0;
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);

  // A stretched/sheared configuration (positive volume).
  Tet deformed = rest;
  deformed[1] += Vec3(0.30, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.22, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.28);

  for (int v = 0; v < 4; ++v) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, v, shape, deformed, mu, lambda);
    const Vec3 numeric = numericForce(shape, deformed, v, mu, lambda);
    EXPECT_NEAR(
        (block.force - numeric).norm(), 0.0, 1e-2 * (1.0 + numeric.norm()))
        << "vertex " << v;
  }
}

//==============================================================================
TEST(VbdNeoHookean, HessianMatchesFiniteDifference)
{
  const double mu = 3000.0;
  const double lambda = 6000.0;
  const Tet rest = referenceTet();
  const vbd::TetRestShape shape = vbd::makeTetRestShape(rest);

  Tet deformed = rest;
  deformed[1] += Vec3(0.20, 0.05, -0.02);
  deformed[2] += Vec3(-0.04, 0.18, 0.03);
  deformed[3] += Vec3(0.02, -0.06, 0.22);

  for (int v = 0; v < 4; ++v) {
    vbd::VertexBlock block;
    vbd::addNeoHookeanTetTerm(block, v, shape, deformed, mu, lambda);
    const Eigen::Matrix3d numeric
        = numericHessian(shape, deformed, v, mu, lambda);
    EXPECT_NEAR((block.hessian - numeric).norm(), 0.0, 1.0)
        << "vertex " << v << "\nanalytic=\n"
        << block.hessian << "\nnumeric=\n"
        << numeric;
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
