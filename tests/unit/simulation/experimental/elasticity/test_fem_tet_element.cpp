/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/detail/deformable_elasticity/fem_tet_element.hpp>

#include <gtest/gtest.h>

#include <array>
#include <functional>

#include <cmath>

namespace fem = dart::simulation::experimental::detail::deformable_elasticity;

namespace {

using Nodes = std::array<Eigen::Vector3d, 4>;

//==============================================================================
// The canonical rest tetrahedron (det Dm = 1, rest volume 1/6).
Nodes restTetrahedron()
{
  return {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0)};
}

//==============================================================================
double energyAt(
    const Nodes& x,
    const fem::TetRestShape& rest,
    const fem::LameParameters& lame)
{
  return fem::evaluateStableNeoHookeanTet(
             x[0], x[1], x[2], x[3], rest, lame, /*computeHessian=*/false)
      .energy;
}

//==============================================================================
// Central-difference gradient of the energy w.r.t. the 12 nodal coordinates.
fem::Vector12d finiteGradient(
    const Nodes& x,
    const fem::TetRestShape& rest,
    const fem::LameParameters& lame)
{
  constexpr double h = 1e-6;
  fem::Vector12d grad = fem::Vector12d::Zero();
  for (int node = 0; node < 4; ++node) {
    for (int axis = 0; axis < 3; ++axis) {
      Nodes plus = x;
      Nodes minus = x;
      plus[node][axis] += h;
      minus[node][axis] -= h;
      grad[3 * node + axis]
          = (energyAt(plus, rest, lame) - energyAt(minus, rest, lame))
            / (2.0 * h);
    }
  }
  return grad;
}

//==============================================================================
// Central-difference Hessian via differencing the analytic gradient.
fem::Matrix12d finiteHessian(
    const Nodes& x,
    const fem::TetRestShape& rest,
    const fem::LameParameters& lame)
{
  constexpr double h = 1e-6;
  fem::Matrix12d hess = fem::Matrix12d::Zero();
  for (int node = 0; node < 4; ++node) {
    for (int axis = 0; axis < 3; ++axis) {
      Nodes plus = x;
      Nodes minus = x;
      plus[node][axis] += h;
      minus[node][axis] -= h;
      const fem::Vector12d gradPlus
          = fem::evaluateStableNeoHookeanTet(
                plus[0], plus[1], plus[2], plus[3], rest, lame)
                .gradient;
      const fem::Vector12d gradMinus
          = fem::evaluateStableNeoHookeanTet(
                minus[0], minus[1], minus[2], minus[3], rest, lame)
                .gradient;
      hess.col(3 * node + axis) = (gradPlus - gradMinus) / (2.0 * h);
    }
  }
  return hess;
}

//==============================================================================
// A representative, non-inverted deformed configuration for the FD checks.
Nodes deformedTetrahedron()
{
  return {
      Eigen::Vector3d(0.02, -0.01, 0.03),
      Eigen::Vector3d(1.21, 0.10, -0.04),
      Eigen::Vector3d(-0.05, 0.92, 0.08),
      Eigen::Vector3d(0.11, 0.18, 1.13)};
}

} // namespace

//==============================================================================
TEST(FemTetElement, LameParametersMatchClosedForm)
{
  const fem::LameParameters lame = fem::lameParameters(1.0e5, 0.3);
  EXPECT_NEAR(lame.mu, 1.0e5 / (2.0 * 1.3), 1e-6);
  EXPECT_NEAR(lame.lambda, 1.0e5 * 0.3 / (1.3 * 0.4), 1e-6);
}

//==============================================================================
TEST(FemTetElement, RestShapeVolumeMatchesAnalytic)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  ASSERT_TRUE(shape.valid);
  EXPECT_NEAR(shape.restVolume, 1.0 / 6.0, 1e-12);
  // Bm = Dm^-1 = I for the canonical rest tet.
  EXPECT_TRUE(
      shape.inverseRestEdges.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

//==============================================================================
TEST(FemTetElement, DegenerateRestShapeIsInvalid)
{
  // Four coplanar points have zero rest volume.
  const fem::TetRestShape shape = fem::makeTetRestShape(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0));
  EXPECT_FALSE(shape.valid);
  const fem::TetElementResult result = fem::evaluateStableNeoHookeanTet(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      shape,
      fem::lameParameters(1.0e3, 0.3));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.energy, 0.0);
}

//==============================================================================
TEST(FemTetElement, RestStateHasZeroEnergyAndForce)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);
  const fem::TetElementResult result = fem::evaluateStableNeoHookeanTet(
      rest[0], rest[1], rest[2], rest[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.energy, 0.0, 1e-9);
  EXPECT_LT(result.gradient.cwiseAbs().maxCoeff(), 1e-7);
}

//==============================================================================
TEST(FemTetElement, GradientMatchesFiniteDifference)
{
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.3);

  const fem::TetElementResult result
      = fem::evaluateStableNeoHookeanTet(x[0], x[1], x[2], x[3], shape, lame);
  const fem::Vector12d numeric = finiteGradient(x, shape, lame);

  const double tol = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tol);
}

//==============================================================================
TEST(FemTetElement, HessianMatchesFiniteDifference)
{
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.3);

  const fem::TetElementResult result
      = fem::evaluateStableNeoHookeanTet(x[0], x[1], x[2], x[3], shape, lame);
  const fem::Matrix12d numeric = finiteHessian(x, shape, lame);

  const double tol = 1e-4 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.hessian - numeric).cwiseAbs().maxCoeff(), tol);
  // The Hessian must be symmetric.
  EXPECT_LT(
      (result.hessian - result.hessian.transpose()).cwiseAbs().maxCoeff(),
      1e-9 * (1.0 + result.hessian.cwiseAbs().maxCoeff()));
}

//==============================================================================
TEST(FemTetElement, EnergyIncreasesUnderStretch)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);

  double previous = -1.0;
  for (const double scale : {1.0, 1.1, 1.5, 2.0}) {
    Nodes x = rest;
    for (auto& node : x) {
      node *= scale;
    }
    const double energy = energyAt(x, shape, lame);
    EXPECT_GE(energy, previous);
    if (scale > 1.0) {
      EXPECT_GT(energy, 0.0);
    }
    previous = energy;
  }
}

//==============================================================================
TEST(FemTetElement, InvertedElementStaysFinite)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);

  // Reflect the fourth node below the base plane so det F < 0 (inverted).
  Nodes inverted = rest;
  inverted[3].z() = -1.0;
  const Eigen::Matrix3d f = fem::deformationGradient(
      inverted[0],
      inverted[1],
      inverted[2],
      inverted[3],
      shape.inverseRestEdges);
  ASSERT_LT(f.determinant(), 0.0);

  const fem::TetElementResult result = fem::evaluateStableNeoHookeanTet(
      inverted[0], inverted[1], inverted[2], inverted[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.energy));
  EXPECT_GT(result.energy, 0.0);
  EXPECT_TRUE(result.gradient.allFinite());
  EXPECT_TRUE(result.hessian.allFinite());
}

//==============================================================================
TEST(FemTetElement, ZeroPoissonRatioStaysFinite)
{
  // lambda -> 0; the kernel must not divide by lambda (alpha would diverge).
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.0);
  EXPECT_NEAR(lame.lambda, 0.0, 1e-12);

  const fem::TetElementResult result
      = fem::evaluateStableNeoHookeanTet(x[0], x[1], x[2], x[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.energy));
  const fem::Vector12d numeric = finiteGradient(x, shape, lame);
  const double tol = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tol);
}
