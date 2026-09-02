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

#include <dart/simulation/detail/deformable_elasticity/fem_tet_element.hpp>

#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <limits>
#include <stdexcept>

#include <cmath>

namespace fem = dart::simulation::detail::deformable_elasticity;

namespace {

using Nodes = std::array<Eigen::Vector3d, 4>;

struct PhysicalLameParameters
{
  double mu;
  double lambda;
};

//==============================================================================
// Independent engineering conversion used by the small-strain Hooke oracle.
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
double finiteMaterialEnergyHessian(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b,
    const fem::LameParameters& model)
{
  // The rest-zeroed log-barrier energy is assembled from O(h) terms that
  // cancel to O(h^2). A 1e-4 step keeps that roundoff below the truncation
  // error while remaining firmly in the infinitesimal regime.
  constexpr double h = 1e-4;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const auto energy = [&](const Eigen::Matrix3d& direction) {
    return fem::stableNeoHookeanEnergyDensity(identity + h * direction, model);
  };
  return (energy(a + b) - energy(a - b) - energy(-a + b) + energy(-a - b))
         / (4.0 * h * h);
}

//==============================================================================
double finiteMaterialStressTangent(
    const Eigen::Matrix3d& a,
    const Eigen::Matrix3d& b,
    const fem::LameParameters& model,
    const double h = 1e-6)
{
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d tangent
      = (fem::stableNeoHookeanFirstPiola(identity + h * b, model)
         - fem::stableNeoHookeanFirstPiola(identity - h * b, model))
        / (2.0 * h);
  return (a.array() * tangent.array()).sum();
}

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
TEST(FemTetElement, LameParametersRemainStandardPhysicalPair)
{
  constexpr double youngsModulus = 1.0e5;
  for (const double poissonRatio : {-0.5, 0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const PhysicalLameParameters expected
        = physicalLame(youngsModulus, poissonRatio);
    const fem::LameParameters actual
        = fem::lameParameters(youngsModulus, poissonRatio);
    EXPECT_NEAR(actual.mu, expected.mu, 1e-12 * expected.mu);
    EXPECT_NEAR(
        actual.lambda,
        expected.lambda,
        1e-12 * (1.0 + std::abs(expected.lambda)));
  }
}

//==============================================================================
TEST(FemTetElement, StableNeoHookeanCoefficientConversionMatchesFormulas)
{
  constexpr double youngsModulus = 1.0e5;
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const PhysicalLameParameters physical
        = physicalLame(youngsModulus, poissonRatio);
    const fem::LameParameters model
        = fem::stableNeoHookeanParameters(youngsModulus, poissonRatio);
    EXPECT_NEAR(model.mu, (4.0 / 3.0) * physical.mu, 1e-12 * model.mu);
    EXPECT_NEAR(
        model.lambda,
        physical.lambda + (5.0 / 6.0) * physical.mu,
        1e-12 * (1.0 + std::abs(model.lambda)));
  }
}

//==============================================================================
TEST(FemTetElement, PublicNonlinearTetSelectorRejectsAuxeticMaterials)
{
  for (const double poissonRatio : {-0.9, -0.5, -1e-12}) {
    SCOPED_TRACE(poissonRatio);
    EXPECT_FALSE(fem::isTetMaterialPoissonRatioSupported(poissonRatio));
    EXPECT_THROW(
        static_cast<void>(fem::stableNeoHookeanParameters(1.0e5, poissonRatio)),
        std::invalid_argument);
    EXPECT_THROW(
        static_cast<void>(fem::tetMaterialParameters(
            1.0e5,
            poissonRatio,
            /*useFixedCorotational=*/false)),
        std::invalid_argument);
    EXPECT_THROW(
        static_cast<void>(fem::tetMaterialParameters(
            1.0e5,
            poissonRatio,
            /*useFixedCorotational=*/true)),
        std::invalid_argument);
  }
  for (const double poissonRatio :
       {0.5, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(poissonRatio);
    EXPECT_FALSE(fem::isTetMaterialPoissonRatioSupported(poissonRatio));
    EXPECT_THROW(
        static_cast<void>(fem::tetMaterialParameters(
            1.0e5,
            poissonRatio,
            /*useFixedCorotational=*/false)),
        std::invalid_argument);
  }
}

//==============================================================================
TEST(FemTetElement, TetMaterialParameterSelectorUsesModelSpecificConversion)
{
  constexpr double youngsModulus = 1.0e5;
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const fem::LameParameters physical
        = fem::lameParameters(youngsModulus, poissonRatio);
    const fem::LameParameters stable
        = fem::stableNeoHookeanParameters(youngsModulus, poissonRatio);
    const fem::LameParameters selectedFixed = fem::tetMaterialParameters(
        youngsModulus, poissonRatio, /*useFixedCorotational=*/true);
    const fem::LameParameters selectedStable = fem::tetMaterialParameters(
        youngsModulus, poissonRatio, /*useFixedCorotational=*/false);
    EXPECT_DOUBLE_EQ(selectedFixed.mu, physical.mu);
    EXPECT_DOUBLE_EQ(selectedFixed.lambda, physical.lambda);
    EXPECT_DOUBLE_EQ(selectedStable.mu, stable.mu);
    EXPECT_DOUBLE_EQ(selectedStable.lambda, stable.lambda);
  }
}

//==============================================================================
// The strongly auxetic Smith log model is locally Hookean but develops a lower
// compressed well. This pins the physical reason for the conservative public
// rejection rather than treating it as an arbitrary input-range choice.
TEST(FemTetElement, StronglyAuxeticLogMappingWouldCreateLowerEnergyWell)
{
  constexpr double youngsModulus = 6000.0;
  constexpr double poissonRatio = -0.9;
  const PhysicalLameParameters physical
      = physicalLame(youngsModulus, poissonRatio);
  const fem::LameParameters wouldBeModel{
      (4.0 / 3.0) * physical.mu,
      std::fma(5.0 / 6.0, physical.mu, physical.lambda)};
  constexpr double compressedScale = 0.6;
  const double restEnergy = fem::stableNeoHookeanEnergyDensity(
      Eigen::Matrix3d::Identity(), wouldBeModel);
  const double compressedEnergy = fem::stableNeoHookeanEnergyDensity(
      compressedScale * Eigen::Matrix3d::Identity(), wouldBeModel);
  EXPECT_DOUBLE_EQ(restEnergy, 0.0);
  EXPECT_LT(compressedEnergy, restEnergy);
}

//==============================================================================
TEST(FemTetElement, SupportedLogUniformScaleEnergyLandscapeIsNonnegative)
{
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const fem::LameParameters model
        = fem::stableNeoHookeanParameters(6000.0, poissonRatio);
    for (const double scale : {0.0, 0.2, 0.6, 1.0, 1.4, 2.0}) {
      SCOPED_TRACE(scale);
      const double energy = fem::stableNeoHookeanEnergyDensity(
          scale * Eigen::Matrix3d::Identity(), model);
      EXPECT_GE(energy, -1e-12 * model.mu);
      if (scale != 1.0) {
        EXPECT_GT(energy, 0.0);
      }
    }
  }
}

//==============================================================================
TEST(FemTetElement, StableNeoHookeanExtremeFiniteInputsStayFinite)
{
  const double poissonRatio = std::numeric_limits<double>::denorm_min();
  const fem::LameParameters model
      = fem::stableNeoHookeanParameters(1e100, poissonRatio);
  ASSERT_TRUE(std::isfinite(model.mu));
  ASSERT_TRUE(std::isfinite(model.lambda));
  ASSERT_GT(model.mu, 0.0);
  ASSERT_GT(model.lambda, 0.0);

  Eigen::Matrix3d f = Eigen::Matrix3d::Identity();
  f(0, 0) = 1.1;
  f(1, 1) = 0.9;
  f(0, 1) = 0.05;
  EXPECT_TRUE(std::isfinite(fem::stableNeoHookeanEnergyDensity(f, model)));
  EXPECT_TRUE(fem::stableNeoHookeanFirstPiola(f, model).allFinite());
  EXPECT_TRUE(fem::stableNeoHookeanEnergyHessian(f, model).allFinite());
}

//==============================================================================
// Near nu=0.5, lambdaHat is many orders of magnitude larger than muHat. The
// determinant coefficient must be evaluated from J-1: forming
// lambdaHat*J-(lambdaHat+3*muHat/4) drops the muHat term and leaves nonzero
// rest stress. A trace-free shear also isolates that term in both the analytic
// and finite-difference Hooke tangents.
TEST(FemTetElement, NearIncompressibleRestAndShearTangentAvoidCancellation)
{
  constexpr double youngsModulus = 1.0;
  const double poissonRatio = std::nextafter(0.5, 0.0);
  const PhysicalLameParameters physical
      = physicalLame(youngsModulus, poissonRatio);
  const fem::LameParameters model
      = fem::stableNeoHookeanParameters(youngsModulus, poissonRatio);
  ASSERT_TRUE(std::isfinite(model.mu));
  ASSERT_TRUE(std::isfinite(model.lambda));
  ASSERT_GT(model.lambda / model.mu, 1e14);

  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  EXPECT_DOUBLE_EQ(fem::stableNeoHookeanEnergyDensity(identity, model), 0.0);
  EXPECT_DOUBLE_EQ(
      fem::stableNeoHookeanFirstPiola(identity, model).squaredNorm(), 0.0);

  Eigen::Matrix3d shear = Eigen::Matrix3d::Zero();
  shear(0, 1) = shear(1, 0) = 0.5;
  const Eigen::Map<const fem::Vector9d> shearDirection(shear.data());
  const fem::Matrix9d analytic
      = fem::stableNeoHookeanEnergyHessian(identity, model);
  const double expected = hookeTangent(physical, shear, shear);
  const double analyticTangent = shearDirection.dot(analytic * shearDirection);
  const double finiteDifferenceTangent
      = finiteMaterialStressTangent(shear, shear, model, 1e-10);
  EXPECT_NEAR(analyticTangent, expected, 1e-12 * expected);
  EXPECT_NEAR(finiteDifferenceTangent, expected, 1e-3 * expected);
}

//==============================================================================
// The expected entries come directly from engineering Hooke elasticity. The
// numerical energy/stress derivatives and the kernel's analytic 9x9 Hessian
// are three separate routes, so this catches an internally self-consistent but
// physically mis-parameterized stable Neo-Hookean implementation.
TEST(FemTetElement, StableNeoHookeanSmallStrainTangentMatchesHookeLaw)
{
  constexpr double youngsModulus = 12345.0;
  const auto basis = symmetricStrainBasis();
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const PhysicalLameParameters physical
        = physicalLame(youngsModulus, poissonRatio);
    const fem::LameParameters model
        = fem::stableNeoHookeanParameters(youngsModulus, poissonRatio);
    const fem::Matrix9d analytic = fem::stableNeoHookeanEnergyHessian(
        Eigen::Matrix3d::Identity(), model);
    for (std::size_t row = 0; row < basis.size(); ++row) {
      const Eigen::Map<const fem::Vector9d> rowDirection(basis[row].data());
      for (std::size_t col = 0; col < basis.size(); ++col) {
        SCOPED_TRACE(::testing::Message() << "row=" << row << " col=" << col);
        const Eigen::Map<const fem::Vector9d> colDirection(basis[col].data());
        const double expected = hookeTangent(physical, basis[row], basis[col]);
        const double energyHessian
            = finiteMaterialEnergyHessian(basis[row], basis[col], model);
        const double stressTangent
            = finiteMaterialStressTangent(basis[row], basis[col], model);
        const double analyticHessian
            = rowDirection.dot(analytic * colDirection);
        const double finiteDifferenceTolerance
            = 5e-5 + 2e-5 * std::abs(expected);
        EXPECT_NEAR(energyHessian, expected, finiteDifferenceTolerance);
        EXPECT_NEAR(stressTangent, expected, finiteDifferenceTolerance);
        EXPECT_NEAR(
            analyticHessian, expected, 1e-11 * (1.0 + std::abs(expected)));
      }
    }
  }
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
      fem::stableNeoHookeanParameters(1.0e3, 0.3));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.energy, 0.0);
}

//==============================================================================
TEST(FemTetElement, RestStateHasZeroEnergyAndForce)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::stableNeoHookeanParameters(1.0e4, 0.3);
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
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const fem::LameParameters model
        = fem::stableNeoHookeanParameters(1.0e3, poissonRatio);
    const fem::TetElementResult result = fem::evaluateStableNeoHookeanTet(
        x[0], x[1], x[2], x[3], shape, model);
    const fem::Vector12d numeric = finiteGradient(x, shape, model);

    const double tolerance = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
    EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tolerance);
  }
}

//==============================================================================
TEST(FemTetElement, HessianMatchesFiniteDifference)
{
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  for (const double poissonRatio : {0.0, 0.3}) {
    SCOPED_TRACE(poissonRatio);
    const fem::LameParameters model
        = fem::stableNeoHookeanParameters(1.0e3, poissonRatio);
    const fem::TetElementResult result = fem::evaluateStableNeoHookeanTet(
        x[0], x[1], x[2], x[3], shape, model);
    const fem::Matrix12d numeric = finiteHessian(x, shape, model);

    const double tolerance = 1e-4 * (1.0 + numeric.cwiseAbs().maxCoeff());
    EXPECT_LT((result.hessian - numeric).cwiseAbs().maxCoeff(), tolerance);
    // The Hessian must be symmetric.
    EXPECT_LT(
        (result.hessian - result.hessian.transpose()).cwiseAbs().maxCoeff(),
        1e-9 * (1.0 + result.hessian.cwiseAbs().maxCoeff()));
  }
}

//==============================================================================
TEST(FemTetElement, EnergyIncreasesUnderStretch)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::stableNeoHookeanParameters(1.0e4, 0.3);

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
  const fem::LameParameters lame = fem::stableNeoHookeanParameters(1.0e4, 0.3);

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
TEST(FemTetElement, EngineeringZeroPoissonRatioStaysFinite)
{
  // Smith's model coefficients remain finite and nonzero at engineering nu=0.
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::stableNeoHookeanParameters(1.0e3, 0.0);
  EXPECT_GT(lame.lambda, 0.0);

  const fem::TetElementResult result
      = fem::evaluateStableNeoHookeanTet(x[0], x[1], x[2], x[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.energy));
  const fem::Vector12d numeric = finiteGradient(x, shape, lame);
  const double tol = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tol);
}

//==============================================================================
TEST(FemTetElement, RawZeroModelLambdaStaysFinite)
{
  // This does not represent engineering nu=0, but the kernel's algebraic
  // continuation at a raw lambdaHat of zero remains a supported safety edge.
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters model{1000.0, 0.0};
  const fem::TetElementResult result
      = fem::evaluateStableNeoHookeanTet(x[0], x[1], x[2], x[3], shape, model);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.energy));
  EXPECT_TRUE(result.gradient.allFinite());
  EXPECT_TRUE(result.hessian.allFinite());
  const fem::Vector12d numeric = finiteGradient(x, shape, model);
  const double tolerance = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tolerance);
}

//==============================================================================
// Fixed-corotational material tests.
namespace {

double fcrEnergyAt(
    const Nodes& x,
    const fem::TetRestShape& rest,
    const fem::LameParameters& lame)
{
  return fem::evaluateFixedCorotationalTet(
             x[0], x[1], x[2], x[3], rest, lame, /*computeHessian=*/false)
      .energy;
}

fem::Vector12d fcrFiniteGradient(
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
          = (fcrEnergyAt(plus, rest, lame) - fcrEnergyAt(minus, rest, lame))
            / (2.0 * h);
    }
  }
  return grad;
}

// Central-difference Hessian via differencing the analytic fixed-corotational
// gradient (the gold-standard check for the exact element Hessian).
fem::Matrix12d fcrFiniteHessian(
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
          = fem::evaluateFixedCorotationalTet(
                plus[0], plus[1], plus[2], plus[3], rest, lame)
                .gradient;
      const fem::Vector12d gradMinus
          = fem::evaluateFixedCorotationalTet(
                minus[0], minus[1], minus[2], minus[3], rest, lame)
                .gradient;
      hess.col(3 * node + axis) = (gradPlus - gradMinus) / (2.0 * h);
    }
  }
  return hess;
}

} // namespace

//==============================================================================
TEST(FemTetElement, FixedCorotationalRestStateHasZeroEnergyAndForce)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);
  const fem::TetElementResult result = fem::evaluateFixedCorotationalTet(
      rest[0], rest[1], rest[2], rest[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.energy, 0.0, 1e-9);
  EXPECT_LT(result.gradient.cwiseAbs().maxCoeff(), 1e-7);
}

//==============================================================================
// The corotational property: a rigid rotation of the rest tetrahedron stores no
// strain energy and exerts no force (unlike a linear model). This is what the
// polar-decomposition rotation buys.
TEST(FemTetElement, FixedCorotationalIsInvariantUnderRotation)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);

  const Eigen::Matrix3d rotation
      = Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.3, -0.7, 0.5).normalized())
            .toRotationMatrix();
  Nodes rotated = rest;
  for (auto& node : rotated) {
    node = rotation * node;
  }

  const fem::TetElementResult result = fem::evaluateFixedCorotationalTet(
      rotated[0], rotated[1], rotated[2], rotated[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.energy, 0.0, 1e-8);
  EXPECT_LT(result.gradient.cwiseAbs().maxCoeff(), 1e-6);
}

//==============================================================================
TEST(FemTetElement, FixedCorotationalGradientMatchesFiniteDifference)
{
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.3);

  const fem::TetElementResult result
      = fem::evaluateFixedCorotationalTet(x[0], x[1], x[2], x[3], shape, lame);
  const fem::Vector12d numeric = fcrFiniteGradient(x, shape, lame);

  const double tol = 1e-5 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((result.gradient - numeric).cwiseAbs().maxCoeff(), tol);
}

//==============================================================================
// The exact fixed-corotational element Hessian (rotation-gradient form) matches
// the finite-difference Hessian of the analytic gradient. Unlike the
// Gauss-Newton approximation it is generally indefinite (exactly like the IPC
// paper's per-element Hessian, which the solver PSD-projects), so this checks
// the value, not definiteness.
TEST(FemTetElement, FixedCorotationalHessianMatchesFiniteDifference)
{
  const Nodes rest = restTetrahedron();
  const Nodes x = deformedTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.3);

  const fem::Matrix12d h
      = fem::evaluateFixedCorotationalTet(x[0], x[1], x[2], x[3], shape, lame)
            .hessian;
  const fem::Matrix12d numeric = fcrFiniteHessian(x, shape, lame);

  EXPECT_LT(
      (h - h.transpose()).cwiseAbs().maxCoeff(),
      1e-9 * (1.0 + h.cwiseAbs().maxCoeff()));
  const double tol = 1e-4 * (1.0 + numeric.cwiseAbs().maxCoeff());
  EXPECT_LT((h - numeric).cwiseAbs().maxCoeff(), tol);
}

//==============================================================================
// The rotation-gradient Hessian is only defined for non-inverted elements; a
// severely inverted element falls back to the always-symmetric Gauss-Newton
// Hessian and must still produce a finite, symmetric block (the solver then
// PSD-projects it) rather than a NaN.
TEST(FemTetElement, FixedCorotationalInvertedElementHessianStaysFinite)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e3, 0.3);

  // Reflect node 3 through the opposite face so the element is inverted (J <
  // 0).
  Nodes inverted = rest;
  inverted[3] = Eigen::Vector3d(0.0, 0.0, -1.0);
  const fem::TetElementResult result = fem::evaluateFixedCorotationalTet(
      inverted[0], inverted[1], inverted[2], inverted[3], shape, lame);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.hessian.allFinite());
  EXPECT_LT(
      (result.hessian - result.hessian.transpose()).cwiseAbs().maxCoeff(),
      1e-9 * (1.0 + result.hessian.cwiseAbs().maxCoeff()));
}

//==============================================================================
// Under a pure uniform scale F = c*I (rotation R = I), the fixed-corotational
// energy density has the closed form 3*mu*(c - 1)^2 + (lambda/2)*(c^3 - 1)^2.
// Scaling every rest node by c about the origin produces exactly F = c*I, so
// the element energy must equal restVolume times that closed form. This pins
// the absolute energy value, which the finite-difference gradient test cannot
// (it only checks the energy's derivative).
TEST(FemTetElement, FixedCorotationalEnergyMatchesUniformScaleClosedForm)
{
  const Nodes rest = restTetrahedron();
  const fem::TetRestShape shape
      = fem::makeTetRestShape(rest[0], rest[1], rest[2], rest[3]);
  const fem::LameParameters lame = fem::lameParameters(1.0e4, 0.3);

  for (const double c : {0.92, 1.0, 1.07}) {
    Nodes scaled = rest;
    for (auto& node : scaled) {
      node *= c;
    }
    const fem::TetElementResult result = fem::evaluateFixedCorotationalTet(
        scaled[0], scaled[1], scaled[2], scaled[3], shape, lame);
    ASSERT_TRUE(result.valid);
    const double expectedDensity
        = 3.0 * lame.mu * (c - 1.0) * (c - 1.0)
          + 0.5 * lame.lambda * (c * c * c - 1.0) * (c * c * c - 1.0);
    EXPECT_NEAR(
        result.energy,
        shape.restVolume * expectedDensity,
        1e-9 * (1.0 + std::abs(shape.restVolume * expectedDensity)));
  }
}
