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

#include <dart/simulation/detail/deformable_vbd/stepper.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;
const Vec3 kGravity(0.0, -10.0, 0.0);

struct Body
{
  std::vector<Vec3> positions;
  std::vector<Vec3> velocities;
  std::vector<Vec3> previousVelocities;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<vbd::SpringElement> springs;
  vbd::VertexColoring coloring;
  vbd::SpringAdjacency adjacency;
  double stiffness = 500.0;
  double timeStep = 0.01;

  void finalize()
  {
    coloring = vbd::colorSprings(positions.size(), springs);
    adjacency = vbd::SpringAdjacency::build(positions.size(), springs);
    velocities.assign(positions.size(), Vec3::Zero());
    previousVelocities.assign(positions.size(), Vec3::Zero());
  }
};

// A vertical chain hanging from a pinned top vertex.
Body makeHangingChain(std::size_t count, double spacing, double restLength)
{
  Body body;
  for (std::size_t i = 0; i < count; ++i) {
    body.positions.emplace_back(0.0, -spacing * static_cast<double>(i), 0.0);
    body.masses.push_back(1.0);
    body.fixed.push_back(i == 0 ? 1u : 0u);
  }
  for (std::size_t i = 0; i + 1 < count; ++i) {
    body.springs.push_back(
        {static_cast<std::uint32_t>(i),
         static_cast<std::uint32_t>(i + 1),
         restLength});
  }
  body.finalize();
  return body;
}

} // namespace

//==============================================================================
TEST(VbdStepper, FreeFallParticleMatchesImplicitEuler)
{
  Body body;
  body.positions = {Vec3(0.0, 0.0, 0.0)};
  body.masses = {1.0};
  body.fixed = {0u};
  body.fixed[0] = 0u; // free
  body.finalize();

  vbd::VbdStepOptions options;
  options.iterations = 5;

  // Implicit-Euler oracle: v += h g; x += h v (exact when there are no forces
  // other than gravity, which is exactly the no-spring case).
  Vec3 refX(0.0, 0.0, 0.0);
  Vec3 refV(0.0, 0.0, 0.0);

  bool hasPrev = false;
  for (int step = 0; step < 6; ++step) {
    vbd::vbdStepMassSpring(
        body.positions,
        body.velocities,
        body.previousVelocities,
        body.masses,
        body.fixed,
        body.springs,
        body.stiffness,
        kGravity,
        body.timeStep,
        body.coloring,
        body.adjacency,
        options,
        hasPrev);
    hasPrev = true;

    refV += body.timeStep * kGravity;
    refX += body.timeStep * refV;
    EXPECT_NEAR((body.positions[0] - refX).norm(), 0.0, 1e-9)
        << "step " << step;
    EXPECT_NEAR((body.velocities[0] - refV).norm(), 0.0, 1e-9)
        << "step " << step;
  }
}

//==============================================================================
TEST(VbdStepper, FixedVertexStaysPut)
{
  Body body;
  body.positions = {Vec3(1.0, 2.0, 3.0)};
  body.masses = {1.0};
  body.fixed = {1u};
  body.finalize();

  vbd::VbdStepOptions options;
  vbd::vbdStepMassSpring(
      body.positions,
      body.velocities,
      body.previousVelocities,
      body.masses,
      body.fixed,
      body.springs,
      body.stiffness,
      kGravity,
      body.timeStep,
      body.coloring,
      body.adjacency,
      options,
      false);

  EXPECT_NEAR((body.positions[0] - Vec3(1.0, 2.0, 3.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(body.velocities[0].norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdStepper, HangingChainIsStableAndStretchesDownward)
{
  Body body = makeHangingChain(6, 0.5, 0.5);
  const double initialBottom = body.positions.back().y();

  vbd::VbdStepOptions options;
  options.iterations = 20;
  bool hasPrev = false;
  for (int step = 0; step < 400; ++step) {
    vbd::vbdStepMassSpring(
        body.positions,
        body.velocities,
        body.previousVelocities,
        body.masses,
        body.fixed,
        body.springs,
        body.stiffness,
        kGravity,
        body.timeStep,
        body.coloring,
        body.adjacency,
        options,
        hasPrev);
    hasPrev = true;
    for (const Vec3& p : body.positions) {
      ASSERT_TRUE(p.allFinite()) << "blew up at step " << step;
      ASSERT_LT(p.norm(), 1e3) << "diverged at step " << step;
    }
  }
  // Top stays pinned; gravity stretches the chain so the bottom hangs lower.
  EXPECT_NEAR(body.positions.front().norm(), 0.0, 1e-12);
  EXPECT_LT(body.positions.back().y(), initialBottom);
}

//==============================================================================
TEST(VbdStepper, ChebyshevWithZeroRhoMatchesPlainStep)
{
  Body plain = makeHangingChain(5, 0.5, 0.4);
  Body cheb = makeHangingChain(5, 0.5, 0.4);

  vbd::VbdStepOptions plainOptions;
  plainOptions.iterations = 10;
  plainOptions.useChebyshev = false;

  vbd::VbdStepOptions chebOptions = plainOptions;
  chebOptions.useChebyshev = true;
  chebOptions.chebyshevRho = 0.0; // omega stays 1 -> over-relaxation is a no-op

  vbd::vbdStepMassSpring(
      plain.positions,
      plain.velocities,
      plain.previousVelocities,
      plain.masses,
      plain.fixed,
      plain.springs,
      plain.stiffness,
      kGravity,
      plain.timeStep,
      plain.coloring,
      plain.adjacency,
      plainOptions,
      false);
  vbd::vbdStepMassSpring(
      cheb.positions,
      cheb.velocities,
      cheb.previousVelocities,
      cheb.masses,
      cheb.fixed,
      cheb.springs,
      cheb.stiffness,
      kGravity,
      cheb.timeStep,
      cheb.coloring,
      cheb.adjacency,
      chebOptions,
      false);

  for (std::size_t i = 0; i < plain.positions.size(); ++i) {
    EXPECT_NEAR((plain.positions[i] - cheb.positions[i]).norm(), 0.0, 1e-12)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdStepper, ChebyshevConvergesToSameStepSolution)
{
  // With enough iterations, a Chebyshev-accelerated step and a plain step reach
  // the same implicit-Euler solution for the first step.
  Body plain = makeHangingChain(6, 0.5, 0.4);
  Body cheb = makeHangingChain(6, 0.5, 0.4);

  vbd::VbdStepOptions plainOptions;
  plainOptions.iterations = 400;

  vbd::VbdStepOptions chebOptions;
  chebOptions.iterations = 400;
  chebOptions.useChebyshev = true;
  chebOptions.chebyshevRho = 0.9;

  vbd::vbdStepMassSpring(
      plain.positions,
      plain.velocities,
      plain.previousVelocities,
      plain.masses,
      plain.fixed,
      plain.springs,
      plain.stiffness,
      kGravity,
      plain.timeStep,
      plain.coloring,
      plain.adjacency,
      plainOptions,
      false);
  vbd::vbdStepMassSpring(
      cheb.positions,
      cheb.velocities,
      cheb.previousVelocities,
      cheb.masses,
      cheb.fixed,
      cheb.springs,
      cheb.stiffness,
      kGravity,
      cheb.timeStep,
      cheb.coloring,
      cheb.adjacency,
      chebOptions,
      false);

  for (std::size_t i = 0; i < plain.positions.size(); ++i) {
    EXPECT_NEAR((plain.positions[i] - cheb.positions[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdStepper, AdaptiveInitConvergesToSameStepSolution)
{
  // The warm start must not change the converged step result, only the path.
  Body adaptive = makeHangingChain(6, 0.5, 0.4);
  Body cold = makeHangingChain(6, 0.5, 0.4);

  vbd::VbdStepOptions adaptiveOptions;
  adaptiveOptions.iterations = 400;
  adaptiveOptions.useAdaptiveInit = true;

  vbd::VbdStepOptions coldOptions;
  coldOptions.iterations = 400;
  coldOptions.useAdaptiveInit = false;

  vbd::vbdStepMassSpring(
      adaptive.positions,
      adaptive.velocities,
      adaptive.previousVelocities,
      adaptive.masses,
      adaptive.fixed,
      adaptive.springs,
      adaptive.stiffness,
      kGravity,
      adaptive.timeStep,
      adaptive.coloring,
      adaptive.adjacency,
      adaptiveOptions,
      false);
  vbd::vbdStepMassSpring(
      cold.positions,
      cold.velocities,
      cold.previousVelocities,
      cold.masses,
      cold.fixed,
      cold.springs,
      cold.stiffness,
      kGravity,
      cold.timeStep,
      cold.coloring,
      cold.adjacency,
      coldOptions,
      false);

  for (std::size_t i = 0; i < adaptive.positions.size(); ++i) {
    EXPECT_NEAR((adaptive.positions[i] - cold.positions[i]).norm(), 0.0, 1e-6)
        << "vertex " << i;
  }
}

//==============================================================================
TEST(VbdStepper, PreviousVelocitiesAdvanceToPreStepVelocities)
{
  Body body = makeHangingChain(4, 0.5, 0.4);
  // Give the body a nonzero starting velocity.
  for (std::size_t i = 0; i < body.velocities.size(); ++i) {
    body.velocities[i] = Vec3(0.0, -0.1, 0.0);
  }
  const std::vector<Vec3> preStep = body.velocities;

  vbd::VbdStepOptions options;
  vbd::vbdStepMassSpring(
      body.positions,
      body.velocities,
      body.previousVelocities,
      body.masses,
      body.fixed,
      body.springs,
      body.stiffness,
      kGravity,
      body.timeStep,
      body.coloring,
      body.adjacency,
      options,
      true);

  for (std::size_t i = 0; i < body.previousVelocities.size(); ++i) {
    EXPECT_NEAR((body.previousVelocities[i] - preStep[i]).norm(), 0.0, 1e-12)
        << "vertex " << i;
  }
}
