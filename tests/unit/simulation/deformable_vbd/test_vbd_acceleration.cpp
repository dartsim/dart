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

#include <dart/simulation/detail/deformable_vbd/acceleration.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {
using Vec3 = Eigen::Vector3d;
const Vec3 kGravity(0.0, -10.0, 0.0);
} // namespace

//==============================================================================
TEST(VbdAcceleration, FirstStepFallsBackToFullInertia)
{
  const Vec3 x(1.0, 2.0, 3.0);
  const Vec3 v(0.1, -0.2, 0.05);
  const double h = 0.01;
  const Vec3 init = vbd::adaptiveInitialPosition(
      x, v, Vec3::Zero(), kGravity, h, /*hasPreviousVelocity=*/false);
  const Vec3 fullInertia = x + h * v + h * h * kGravity;
  EXPECT_NEAR((init - fullInertia).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdAcceleration, FreeFallReachesFullInertia)
{
  // Previous acceleration aligned with gravity and at least as large -> the
  // clamp saturates at ||g|| and the guess equals the full inertial prediction.
  const Vec3 x(0.0, 5.0, 0.0);
  const double h = 0.01;
  const Vec3 v(0.0, -0.5, 0.0);
  const Vec3 vPrev = v - h * kGravity; // so a_prev = (v - vPrev)/h = gravity
  const Vec3 init = vbd::adaptiveInitialPosition(
      x, v, vPrev, kGravity, h, /*hasPreviousVelocity=*/true);
  const Vec3 fullInertia = x + h * v + h * h * kGravity;
  EXPECT_NEAR((init - fullInertia).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdAcceleration, SupportedBodyDropsGravityStep)
{
  // Previous acceleration opposes gravity (the body was decelerating upward as
  // if resting on support) -> projection is negative -> clamp to 0 -> the guess
  // omits the gravity step.
  const Vec3 x(0.0, 0.0, 0.0);
  const double h = 0.01;
  const Vec3 v(0.0, 0.0, 0.0);
  const Vec3 vPrev(
      0.0, -0.3, 0.0); // a_prev = (0 - (-0.3))/h = +y (anti-gravity)
  const Vec3 init = vbd::adaptiveInitialPosition(
      x, v, vPrev, kGravity, h, /*hasPreviousVelocity=*/true);
  const Vec3 noGravityStep = x + h * v;
  EXPECT_NEAR((init - noGravityStep).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdAcceleration, PartialAlignmentBlendsBetweenBounds)
{
  const Vec3 x(0.0, 0.0, 0.0);
  const double h = 0.01;
  const Vec3 v(0.0, 0.0, 0.0);
  // a_prev along -y with magnitude 4 < ||g|| = 10 -> blend factor 4.
  const Vec3 vPrev = v - h * Vec3(0.0, -4.0, 0.0);
  const Vec3 init = vbd::adaptiveInitialPosition(
      x, v, vPrev, kGravity, h, /*hasPreviousVelocity=*/true);
  const Vec3 expected = x + h * v + h * h * Vec3(0.0, -1.0, 0.0) * 4.0;
  EXPECT_NEAR((init - expected).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdAcceleration, ChebyshevOmegaFollowsRecurrence)
{
  const double rho = 0.5;
  const double omega1 = vbd::chebyshevOmega(1, rho, 0.0);
  EXPECT_DOUBLE_EQ(omega1, 1.0);

  const double omega2 = vbd::chebyshevOmega(2, rho, omega1);
  EXPECT_NEAR(omega2, 2.0 / (2.0 - 0.25), 1e-12);

  const double omega3 = vbd::chebyshevOmega(3, rho, omega2);
  EXPECT_NEAR(omega3, 4.0 / (4.0 - 0.25 * omega2), 1e-12);

  // Weights are over-relaxing (>1) and bounded for a valid spectral radius.
  double omega = 1.0;
  for (std::size_t n = 1; n <= 50; ++n) {
    omega = vbd::chebyshevOmega(n, rho, omega);
    EXPECT_GE(omega, 1.0);
    EXPECT_LT(omega, 2.0);
  }
}

//==============================================================================
TEST(VbdAcceleration, ChebyshevApplyIsAffineAndIdentityAtUnitWeight)
{
  const Vec3 swept(1.0, 2.0, 3.0);
  const Vec3 twoBack(0.5, 0.5, 0.5);

  // omega <= 1 leaves the swept iterate unchanged.
  EXPECT_NEAR(
      (vbd::applyChebyshev(1.0, swept, twoBack) - swept).norm(), 0.0, 1e-12);

  // omega > 1 over-relaxes along (swept - twoBack).
  const double omega = 1.5;
  const Vec3 expected = omega * (swept - twoBack) + twoBack;
  EXPECT_NEAR(
      (vbd::applyChebyshev(omega, swept, twoBack) - expected).norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(VbdAcceleration, RayleighDampingAddsScaledHessianAndForce)
{
  vbd::VertexBlock block;
  block.force = Vec3(1.0, 2.0, 3.0);
  block.hessian = Eigen::Matrix3d::Identity() * 10.0;

  Eigen::Matrix3d elastic;
  elastic << 4.0, 1.0, 0.0, 1.0, 5.0, 2.0, 0.0, 2.0, 6.0;
  const Vec3 displacement(0.1, -0.2, 0.05);
  const double kd = 1e-3;
  const double h = 0.01;

  const Eigen::Matrix3d baseHessian = block.hessian;
  const Vec3 baseForce = block.force;
  vbd::addRayleighDamping(block, elastic, displacement, kd, h);

  const double scale = kd / h;
  EXPECT_NEAR(
      (block.hessian - (baseHessian + scale * elastic)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (block.force - (baseForce - scale * (elastic * displacement))).norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(VbdAcceleration, RayleighDampingIsDissipative)
{
  // The damping force does non-positive work against the displacement when the
  // elastic Hessian is positive semidefinite.
  vbd::VertexBlock block;
  Eigen::Matrix3d elastic;
  elastic << 4.0, 1.0, 0.0, 1.0, 5.0, 2.0, 0.0, 2.0, 6.0; // SPD
  const Vec3 displacement(0.1, -0.2, 0.05);

  vbd::VertexBlock damped = block;
  vbd::addRayleighDamping(damped, elastic, displacement, 1e-3, 0.01);
  // Only the damping force is present (block started at zero force).
  EXPECT_LE(damped.force.dot(displacement), 0.0);
}

//==============================================================================
TEST(VbdAcceleration, RayleighDampingNoOpForZeroCoefficient)
{
  vbd::VertexBlock block;
  block.force = Vec3(1.0, 0.0, 0.0);
  block.hessian = Eigen::Matrix3d::Identity();
  vbd::addRayleighDamping(
      block, Eigen::Matrix3d::Identity(), Vec3(1.0, 1.0, 1.0), 0.0, 0.01);
  EXPECT_NEAR((block.force - Vec3(1.0, 0.0, 0.0)).norm(), 0.0, 1e-15);
  EXPECT_NEAR((block.hessian - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-15);
}
