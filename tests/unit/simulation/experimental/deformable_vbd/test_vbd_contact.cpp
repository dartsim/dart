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

#include <dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/experimental/detail/deformable_vbd/contact_kernel.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <functional>
#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

double planeEnergy(const Vec3& x, const vbd::ContactPlane& plane)
{
  const double penetration = plane.offset - plane.normal.dot(x);
  if (penetration <= 0.0) {
    return 0.0;
  }
  return 0.5 * plane.stiffness * penetration * penetration;
}

Vec3 numericGradient(
    const std::function<double(const Vec3&)>& energy,
    const Vec3& at,
    double eps = 1e-6)
{
  Vec3 grad = Vec3::Zero();
  for (int d = 0; d < 3; ++d) {
    Vec3 plus = at;
    Vec3 minus = at;
    plus[d] += eps;
    minus[d] -= eps;
    grad[d] = (energy(plus) - energy(minus)) / (2.0 * eps);
  }
  return grad;
}

} // namespace

//==============================================================================
TEST(VbdContact, PenaltyForceMatchesFiniteDifferenceWhenPenetrating)
{
  vbd::ContactPlane plane;
  plane.normal = Vec3(0.0, 1.0, 0.0);
  plane.offset = 0.0;
  plane.stiffness = 1.0e4;
  const Vec3 position(0.3, -0.2, 0.1); // below the plane -> penetrating

  vbd::VertexBlock block;
  vbd::addHalfSpacePenaltyContact(block, position, plane);

  const auto energy = [&](const Vec3& x) {
    return planeEnergy(x, plane);
  };
  const Vec3 numericForce = -numericGradient(energy, position);
  EXPECT_NEAR((block.force - numericForce).norm(), 0.0, 1e-3);
  // Force pushes out along +normal.
  EXPECT_GT(block.force.dot(plane.normal), 0.0);
}

//==============================================================================
TEST(VbdContact, InactiveAbovePlaneAndHessianIsPsd)
{
  vbd::ContactPlane plane;
  plane.normal = Vec3(0.0, 1.0, 0.0);
  plane.offset = 0.0;
  plane.stiffness = 1.0e4;

  vbd::VertexBlock above;
  vbd::addHalfSpacePenaltyContact(above, Vec3(0.0, 0.5, 0.0), plane);
  EXPECT_NEAR(above.force.norm(), 0.0, 1e-15);
  EXPECT_NEAR(above.hessian.norm(), 0.0, 1e-15);

  vbd::VertexBlock penetrating;
  vbd::addHalfSpacePenaltyContact(penetrating, Vec3(0.0, -0.1, 0.0), plane);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(penetrating.hessian);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9);
}

//==============================================================================
// A free particle dropped onto a ground plane rests on it (small penetration)
// rather than tunneling through.
TEST(VbdContact, ParticleRestsOnGround)
{
  std::vector<Vec3> positions = {Vec3(0.0, 0.5, 0.0)};
  std::vector<double> masses = {1.0};
  std::vector<std::uint8_t> fixed = {0u};
  fixed[0] = 0u;
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(1, springs);
  const auto adjacency = vbd::SpringAdjacency::build(1, springs);

  vbd::ContactPlane ground;
  ground.normal = Vec3(0.0, 1.0, 0.0);
  ground.offset = 0.0;
  ground.stiffness = 1.0e5;
  const std::vector<vbd::ContactPlane> planes = {ground};

  const Vec3 gravity(0.0, -9.81, 0.0);
  const double h = 0.01;
  std::vector<Vec3> velocity = {Vec3::Zero()};
  vbd::BlockDescentOptions options;
  options.iterations = 30;

  for (int step = 0; step < 300; ++step) {
    std::vector<Vec3> inertialTargets = positions;
    inertialTargets[0] = positions[0] + h * velocity[0] + h * h * gravity;
    const std::vector<Vec3> previous = positions;
    vbd::blockDescentMassSpringGround(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        0.0,
        h,
        planes,
        coloring,
        adjacency,
        options);
    velocity[0] = (positions[0] - previous[0]) / h;
    ASSERT_TRUE(positions[0].allFinite()) << "blew up at step " << step;
  }

  // Rests near the plane: not tunneled (penetration bounded) and not floating.
  EXPECT_GT(positions[0].y(), -0.01);
  EXPECT_LT(positions[0].y(), 0.05);
  EXPECT_LT(std::abs(velocity[0].y()), 0.05);
}

//==============================================================================
// A small pinned spring net sagging under gravity onto a ground plane stays
// above the plane.
TEST(VbdContact, SpringNetRestsAboveGround)
{
  const int side = 5;
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<vbd::SpringElement> springs;
  const auto index = [side](int r, int c) {
    return static_cast<std::uint32_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      positions.emplace_back(c * 0.2, 0.4, r * 0.2);
      masses.push_back(1.0);
      fixed.push_back((r == 0 && (c == 0 || c == side - 1)) ? 1u : 0u);
    }
  }
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        springs.push_back({index(r, c), index(r, c + 1), 0.2});
      }
      if (r + 1 < side) {
        springs.push_back({index(r, c), index(r + 1, c), 0.2});
      }
    }
  }
  const auto coloring = vbd::colorSprings(positions.size(), springs);
  const auto adjacency = vbd::SpringAdjacency::build(positions.size(), springs);

  vbd::ContactPlane ground;
  ground.normal = Vec3(0.0, 1.0, 0.0);
  ground.offset = 0.0;
  ground.stiffness = 5.0e4;
  const std::vector<vbd::ContactPlane> planes = {ground};

  const Vec3 gravity(0.0, -9.81, 0.0);
  const double h = 0.01;
  std::vector<Vec3> velocity(positions.size(), Vec3::Zero());
  vbd::BlockDescentOptions options;
  options.iterations = 40;

  for (int step = 0; step < 200; ++step) {
    std::vector<Vec3> inertialTargets = positions;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      if (fixed[i] == 0u) {
        inertialTargets[i] = positions[i] + h * velocity[i] + h * h * gravity;
      }
    }
    const std::vector<Vec3> previous = positions;
    vbd::blockDescentMassSpringGround(
        positions,
        masses,
        fixed,
        inertialTargets,
        springs,
        500.0,
        h,
        planes,
        coloring,
        adjacency,
        options);
    for (std::size_t i = 0; i < positions.size(); ++i) {
      velocity[i] = (positions[i] - previous[i]) / h;
    }
  }

  for (const Vec3& p : positions) {
    ASSERT_TRUE(p.allFinite());
    EXPECT_GT(p.y(), -0.02) << "node tunneled below the ground";
  }
}

//==============================================================================
// In the sticking regime, friction is the gradient of the tangential penalty
// energy 0.5 k_c ||T(x - x^t)||^2, so its force matches finite differences.
TEST(VbdContact, FrictionStickingForceMatchesFiniteDifference)
{
  vbd::ContactPlane plane;
  plane.normal = Vec3(0.0, 1.0, 0.0);
  plane.offset = 0.0;
  plane.stiffness = 1.0e4;
  const Vec3 stepStart(0.0, -0.1, 0.0);    // penetrating
  const Vec3 position(0.01, -0.1, -0.005); // small tangential move -> sticking

  vbd::VertexBlock block;
  vbd::addHalfSpaceFriction(block, position, stepStart, plane, 0.5);

  const auto energy = [&](const Vec3& x) {
    const Vec3 delta = x - stepStart;
    const Vec3 u = delta - plane.normal.dot(delta) * plane.normal;
    return 0.5 * plane.stiffness * u.squaredNorm();
  };
  const Vec3 numericForce = -numericGradient(energy, position);
  EXPECT_NEAR((block.force - numericForce).norm(), 0.0, 1e-3);
}

//==============================================================================
// In the sliding regime, the friction force magnitude is capped at the Coulomb
// limit mu * lambda.
TEST(VbdContact, FrictionSlidingForceIsCoulombCapped)
{
  vbd::ContactPlane plane;
  plane.normal = Vec3(0.0, 1.0, 0.0);
  plane.offset = 0.0;
  plane.stiffness = 1.0e4;
  const double penetration = 0.1;
  const double frictionCoeff = 0.5;
  const Vec3 stepStart(0.0, -penetration, 0.0);
  const Vec3 position(0.5, -penetration, 0.0); // large tangential move -> slide

  vbd::VertexBlock block;
  vbd::addHalfSpaceFriction(block, position, stepStart, plane, frictionCoeff);

  const double coulomb = frictionCoeff * plane.stiffness * penetration;
  EXPECT_NEAR(block.force.norm(), coulomb, 1e-6);
  // Friction opposes the tangential motion (-x here).
  EXPECT_LT(block.force.x(), 0.0);
}

//==============================================================================
// A particle sliding on the ground is decelerated to rest by kinetic friction.
TEST(VbdContact, KineticFrictionStopsASlidingParticle)
{
  std::vector<Vec3> positions = {Vec3(0.0, -0.001, 0.0)};
  std::vector<double> masses = {1.0};
  std::vector<std::uint8_t> fixed = {0u};
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(1, springs);
  const auto adjacency = vbd::SpringAdjacency::build(1, springs);

  vbd::ContactPlane ground;
  ground.normal = Vec3(0.0, 1.0, 0.0);
  ground.offset = 0.0;
  ground.stiffness = 1.0e5;
  const std::vector<vbd::ContactPlane> planes = {ground};
  const double frictionCoeff = 0.6;

  const Vec3 gravity(0.0, -9.81, 0.0);
  const double h = 0.01;
  std::vector<Vec3> velocity = {Vec3(1.5, 0.0, 0.0)};
  const double initialSpeed = velocity[0].x();
  vbd::BlockDescentOptions options;
  options.iterations = 30;

  for (int step = 0; step < 400; ++step) {
    std::vector<Vec3> inertialTargets = positions;
    inertialTargets[0] = positions[0] + h * velocity[0] + h * h * gravity;
    const std::vector<Vec3> stepStart = positions;
    vbd::blockDescentMassSpringGroundFriction(
        positions,
        masses,
        fixed,
        inertialTargets,
        stepStart,
        springs,
        0.0,
        h,
        planes,
        frictionCoeff,
        coloring,
        adjacency,
        options);
    velocity[0] = (positions[0] - stepStart[0]) / h;
    ASSERT_TRUE(positions[0].allFinite()) << "blew up at step " << step;
  }

  // Kinetic friction dissipates the tangential speed.
  EXPECT_LT(std::abs(velocity[0].x()), 0.1 * initialSpeed);
}
