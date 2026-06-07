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

#include <dart/simulation/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <functional>

#include <cmath>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

//==============================================================================
// Spring elastic energy E = (k/2)(||other - self|| - L)^2, as a function of the
// `self` vertex only (`other` held fixed). Used as the finite-difference
// oracle.
double springEnergy(
    double stiffness,
    double restLength,
    const Eigen::Vector3d& self,
    const Eigen::Vector3d& other)
{
  const double length = (other - self).norm();
  const double stretch = length - restLength;
  return 0.5 * stiffness * stretch * stretch;
}

//==============================================================================
Eigen::Vector3d numericGradient(
    const std::function<double(const Eigen::Vector3d&)>& energy,
    const Eigen::Vector3d& at,
    double eps = 1e-6)
{
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  for (int d = 0; d < 3; ++d) {
    Eigen::Vector3d plus = at;
    Eigen::Vector3d minus = at;
    plus[d] += eps;
    minus[d] -= eps;
    grad[d] = (energy(plus) - energy(minus)) / (2.0 * eps);
  }
  return grad;
}

//==============================================================================
Eigen::Matrix3d numericHessian(
    const std::function<double(const Eigen::Vector3d&)>& energy,
    const Eigen::Vector3d& at,
    double eps = 1e-4)
{
  Eigen::Matrix3d hess = Eigen::Matrix3d::Zero();
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      Eigen::Vector3d pp = at;
      Eigen::Vector3d pm = at;
      Eigen::Vector3d mp = at;
      Eigen::Vector3d mm = at;
      pp[a] += eps;
      pp[b] += eps;
      pm[a] += eps;
      pm[b] -= eps;
      mp[a] -= eps;
      mp[b] += eps;
      mm[a] -= eps;
      mm[b] -= eps;
      hess(a, b) = (energy(pp) - energy(pm) - energy(mp) + energy(mm))
                   / (4.0 * eps * eps);
    }
  }
  return hess;
}

//==============================================================================
double minEigenvalue(const Eigen::Matrix3d& matrix)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(matrix);
  return solver.eigenvalues().minCoeff();
}

} // namespace

//==============================================================================
TEST(VbdVertexBlockKernel, InertiaTermForceAndHessianAreExact)
{
  const double mass = 2.5;
  const double timeStep = 0.01;
  const Eigen::Vector3d position(0.3, -0.7, 1.2);
  const Eigen::Vector3d target(0.1, -0.5, 1.0);

  vbd::VertexBlock block;
  vbd::addInertiaTerm(block, mass, timeStep, position, target);

  const double coeff = mass / (timeStep * timeStep);
  const Eigen::Vector3d expectedForce = -coeff * (position - target);
  EXPECT_NEAR((block.force - expectedForce).norm(), 0.0, 1e-9);
  EXPECT_NEAR(
      (block.hessian - coeff * Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-9);

  // At the target the inertia force vanishes.
  vbd::VertexBlock atTarget;
  vbd::addInertiaTerm(atTarget, mass, timeStep, target, target);
  EXPECT_NEAR(atTarget.force.norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdVertexBlockKernel, SpringForceMatchesFiniteDifferenceWhenStretched)
{
  const double stiffness = 100.0;
  const double restLength = 1.0;
  const Eigen::Vector3d self(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(1.6, 0.2, -0.1); // length ~1.61 > restLength

  vbd::VertexBlock block;
  vbd::addSpringTerm(
      block, stiffness, restLength, self, other, /*clampToPsd=*/false);

  const auto energy = [&](const Eigen::Vector3d& x) {
    return springEnergy(stiffness, restLength, x, other);
  };
  const Eigen::Vector3d numericForce = -numericGradient(energy, self);
  EXPECT_NEAR((block.force - numericForce).norm(), 0.0, 1e-4);
}

//==============================================================================
TEST(VbdVertexBlockKernel, SpringForceMatchesFiniteDifferenceWhenCompressed)
{
  const double stiffness = 75.0;
  const double restLength = 1.5;
  const Eigen::Vector3d self(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(0.8, -0.3, 0.1); // length ~0.86 < restLength

  vbd::VertexBlock block;
  vbd::addSpringTerm(
      block, stiffness, restLength, self, other, /*clampToPsd=*/false);

  const auto energy = [&](const Eigen::Vector3d& x) {
    return springEnergy(stiffness, restLength, x, other);
  };
  const Eigen::Vector3d numericForce = -numericGradient(energy, self);
  EXPECT_NEAR((block.force - numericForce).norm(), 0.0, 1e-4);
}

//==============================================================================
TEST(VbdVertexBlockKernel, UnclampedSpringHessianMatchesFiniteDifference)
{
  const double stiffness = 120.0;
  const double restLength = 1.0;
  const Eigen::Vector3d other(1.7, 0.4, 0.2);

  for (const Eigen::Vector3d& self :
       {Eigen::Vector3d(0.0, 0.0, 0.0),    // stretched
        Eigen::Vector3d(1.1, 0.2, 0.1)}) { // compressed
    vbd::VertexBlock block;
    vbd::addSpringTerm(
        block, stiffness, restLength, self, other, /*clampToPsd=*/false);

    const auto energy = [&](const Eigen::Vector3d& x) {
      return springEnergy(stiffness, restLength, x, other);
    };
    const Eigen::Matrix3d numeric = numericHessian(energy, self);
    EXPECT_NEAR((block.hessian - numeric).norm(), 0.0, 1e-2)
        << "self = " << self.transpose();
  }
}

//==============================================================================
TEST(VbdVertexBlockKernel, ClampedSpringHessianIsPositiveSemidefinite)
{
  const double stiffness = 100.0;
  const double restLength = 2.0;
  const Eigen::Vector3d other(0.5, 0.0, 0.0); // heavily compressed (l=0.5<2.0)
  const Eigen::Vector3d self(0.0, 0.0, 0.0);

  vbd::VertexBlock unclamped;
  vbd::addSpringTerm(
      unclamped, stiffness, restLength, self, other, /*clampToPsd=*/false);
  // The exact energy Hessian is indefinite under heavy compression.
  EXPECT_LT(minEigenvalue(unclamped.hessian), 0.0);

  vbd::VertexBlock clamped;
  vbd::addSpringTerm(
      clamped, stiffness, restLength, self, other, /*clampToPsd=*/true);
  EXPECT_GE(minEigenvalue(clamped.hessian), -1e-9);
}

//==============================================================================
TEST(VbdVertexBlockKernel, ClampedEqualsUnclampedWhenStretched)
{
  const double stiffness = 100.0;
  const double restLength = 1.0;
  const Eigen::Vector3d self(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(2.0, 0.0, 0.0); // stretched, transverse > 0

  vbd::VertexBlock clamped;
  vbd::VertexBlock unclamped;
  vbd::addSpringTerm(clamped, stiffness, restLength, self, other, true);
  vbd::addSpringTerm(unclamped, stiffness, restLength, self, other, false);
  EXPECT_NEAR((clamped.hessian - unclamped.hessian).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdVertexBlockKernel, DegenerateSpringContributesNothing)
{
  vbd::VertexBlock block;
  const Eigen::Vector3d self(0.0, 0.0, 0.0);
  vbd::addSpringTerm(block, 100.0, 1.0, self, self, /*clampToPsd=*/true);
  EXPECT_NEAR(block.force.norm(), 0.0, 1e-15);
  EXPECT_NEAR(block.hessian.norm(), 0.0, 1e-15);
}

//==============================================================================
TEST(VbdVertexBlockKernel, SolveRecoversTheNewtonStep)
{
  vbd::VertexBlock block;
  block.hessian << 4.0, 1.0, 0.5, 1.0, 3.0, 0.2, 0.5, 0.2, 5.0;
  block.force = Eigen::Vector3d(1.0, -2.0, 0.5);

  const Eigen::Vector3d delta = vbd::solveVertexBlock(block);
  // H * delta should reproduce the force.
  EXPECT_NEAR((block.hessian * delta - block.force).norm(), 0.0, 1e-9);
}

//==============================================================================
TEST(VbdVertexBlockKernel, SolveRejectsNonPositiveDefiniteHessian)
{
  vbd::VertexBlock block;
  block.hessian = -Eigen::Matrix3d::Identity();
  block.force = Eigen::Vector3d(1.0, 1.0, 1.0);
  EXPECT_NEAR(vbd::solveVertexBlock(block).norm(), 0.0, 1e-15);

  vbd::VertexBlock nanBlock;
  nanBlock.hessian = Eigen::Matrix3d::Identity();
  nanBlock.force = Eigen::Vector3d(std::nan(""), 0.0, 0.0);
  EXPECT_NEAR(vbd::solveVertexBlock(nanBlock).norm(), 0.0, 1e-15);
}

//==============================================================================
TEST(VbdVertexBlockKernel, RegularizationShortensTheStep)
{
  vbd::VertexBlock block;
  block.hessian = Eigen::Matrix3d::Identity();
  block.force = Eigen::Vector3d(1.0, 0.0, 0.0);

  const Eigen::Vector3d plain = vbd::solveVertexBlock(block, 0.0);
  const Eigen::Vector3d damped = vbd::solveVertexBlock(block, 1.0);
  EXPECT_NEAR(plain.norm(), 1.0, 1e-9);
  EXPECT_NEAR(damped.norm(), 0.5, 1e-9); // 1/(1+1)
  EXPECT_LT(damped.norm(), plain.norm());
}

//==============================================================================
// A single block Newton step on the local objective
//   G_i(x) = (m/2h^2)||x - y||^2 + (k/2)(||other - x|| - L)^2
// must be a descent step (g . delta < 0) and, for a stretched configuration
// where the local Hessian is PD, must strictly reduce G_i.
TEST(VbdVertexBlockKernel, BlockStepReducesLocalObjective)
{
  const double mass = 1.0;
  const double timeStep = 0.05;
  const double stiffness = 200.0;
  const double restLength = 1.0;
  const Eigen::Vector3d target(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(2.4, 0.0, 0.0); // far -> stretched spring
  const Eigen::Vector3d x0(0.2, 0.1, -0.05);

  const auto objective = [&](const Eigen::Vector3d& x) {
    const double inertia
        = 0.5 * mass / (timeStep * timeStep) * (x - target).squaredNorm();
    return inertia + springEnergy(stiffness, restLength, x, other);
  };

  vbd::VertexBlock block;
  vbd::addInertiaTerm(block, mass, timeStep, x0, target);
  vbd::addSpringTerm(block, stiffness, restLength, x0, other, true);

  const Eigen::Vector3d delta = vbd::solveVertexBlock(block);
  // Descent direction: gradient . delta = (-force) . delta < 0.
  EXPECT_LT((-block.force).dot(delta), 0.0);

  const Eigen::Vector3d x1 = x0 + delta;
  EXPECT_LT(objective(x1), objective(x0));
}

//==============================================================================
// Iterating the block solve drives the local residual force to zero
// (converges to the stationary point of the local objective).
TEST(VbdVertexBlockKernel, IteratedBlockSolveConverges)
{
  const double mass = 1.0;
  const double timeStep = 0.05;
  const double stiffness = 200.0;
  const double restLength = 1.0;
  const Eigen::Vector3d target(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(2.4, 0.1, -0.2);
  Eigen::Vector3d x(0.2, 0.1, -0.05);

  double residual = 0.0;
  for (int iteration = 0; iteration < 50; ++iteration) {
    vbd::VertexBlock block;
    vbd::addInertiaTerm(block, mass, timeStep, x, target);
    vbd::addSpringTerm(block, stiffness, restLength, x, other, true);
    residual = block.force.norm();
    x += vbd::solveVertexBlock(block);
  }
  EXPECT_LT(residual, 1e-8);
}
