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

#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/detail/deformable_vbd/contact_kernel.hpp>

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <functional>
#include <vector>

#include <cmath>
#include <cstdint>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace common = dart::common;

namespace {

using Vec3 = Eigen::Vector3d;

template <typename T>
using AllocatorVector = std::vector<T, common::StlAllocator<T>>;

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
TEST(VbdContact, SelfContactAdjacencyUsesProvidedAllocator)
{
  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeListAllocator();
  const auto allocationsBefore = allocator.getAllocationCount();

  vbd::SelfContactAdjacency adjacency(allocator);
  adjacency.reserve(/*vertexCount=*/4, /*candidateCapacity=*/8);

  EXPECT_GT(allocator.getAllocationCount(), allocationsBefore);
  ASSERT_EQ(adjacency.incident.size(), 4u);
  EXPECT_EQ(
      adjacency.incident.get_allocator(),
      common::StlAllocator<vbd::SelfContactAdjacency::IncidentVector>{
          allocator});
  EXPECT_EQ(
      adjacency.incident[0].get_allocator(),
      common::StlAllocator<vbd::SelfContactEntry>{allocator});
}

//==============================================================================
TEST(VbdContact, AvbdNormalForceMatchesPenaltyForPenetratingActiveRow)
{
  vbd::ContactPlane plane;
  plane.normal = Vec3(0.0, 1.0, 0.0);
  plane.offset = 0.0;
  plane.stiffness = 1.0e4;
  const Vec3 position(0.3, -0.2, 0.1);

  vbd::VertexBlock penalty;
  vbd::addHalfSpacePenaltyContact(penalty, position, plane);

  vbd::AvbdScalarRowState row;
  row.stiffness = plane.stiffness;
  row.lambda = 0.0;

  vbd::VertexBlock avbd;
  const double forceMagnitude = vbd::addAvbdHalfSpaceContactNormal(
      avbd,
      position,
      plane,
      row,
      /*previousConstraintValue=*/0.0,
      /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 2000.0);
  EXPECT_NEAR((avbd.force - penalty.force).norm(), 0.0, 1e-12);
  EXPECT_NEAR((avbd.hessian - penalty.hessian).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionTangentForceIsCoulombBounded)
{
  const Vec3 stepStart(0.0, -0.1, 0.0);
  const Vec3 position(0.2, -0.1, 0.0);

  vbd::AvbdHalfSpaceFrictionRow row;
  row.vertex = 0;
  row.stepStartPosition = stepStart;
  row.axis = Vec3::UnitX();
  row.state.stiffness = 100.0;
  row.state.lambda = 0.0;
  row.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::VertexBlock block;
  const double forceMagnitude = vbd::addAvbdHalfSpaceFrictionTangent(
      block, position, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, -5.0);
  EXPECT_NEAR(block.force.x(), -5.0, 1e-12);
  EXPECT_NEAR(block.force.y(), 0.0, 1e-12);
  EXPECT_NEAR(block.force.z(), 0.0, 1e-12);
  EXPECT_NEAR(block.hessian(0, 0), 100.0, 1e-12);
  EXPECT_NEAR(block.hessian(1, 1), 0.0, 1e-12);
  EXPECT_NEAR(block.hessian(2, 2), 0.0, 1e-12);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionTangentUpdatesDualStateWithinBounds)
{
  vbd::AvbdHalfSpaceFrictionRow row;
  row.vertex = 0;
  row.stepStartPosition = Vec3::Zero();
  row.axis = Vec3::UnitX();
  row.state.stiffness = 10.0;
  row.state.lambda = 0.0;
  row.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::AvbdHalfSpaceFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  const vbd::AvbdScalarRowState unclamped
      = vbd::updateAvbdHalfSpaceFrictionTangentRow(
          row.state, Vec3(0.1, 0.0, 0.0), row, options);
  EXPECT_NEAR(unclamped.lambda, -1.0, 1e-12);
  EXPECT_GT(unclamped.stiffness, row.state.stiffness);

  const vbd::AvbdScalarRowState clamped
      = vbd::updateAvbdHalfSpaceFrictionTangentRow(
          row.state, Vec3(1.0, 0.0, 0.0), row, options);
  EXPECT_NEAR(clamped.lambda, -5.0, 1e-12);
  EXPECT_DOUBLE_EQ(clamped.stiffness, row.state.stiffness);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionTangentPairProjectsStaticForceToCone)
{
  vbd::AvbdHalfSpaceFrictionRow rowX;
  rowX.vertex = 0;
  rowX.stepStartPosition = Vec3::Zero();
  rowX.axis = Vec3::UnitX();
  rowX.state.stiffness = 10.0;
  rowX.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::AvbdHalfSpaceFrictionRow rowY;
  rowY.vertex = 0;
  rowY.stepStartPosition = Vec3::Zero();
  rowY.axis = Vec3::UnitY();
  rowY.state.stiffness = 10.0;
  rowY.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::AvbdHalfSpaceFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_TRUE(vbd::avbdFrictionPreviousDualInsideCone(rowX, rowY));
  bool clamped = false;
  const Eigen::Vector2d force = vbd::avbdHalfSpaceFrictionTangentPairForce(
      Vec3(1.0, 1.0, 0.0), rowX, rowY, options, &clamped);

  EXPECT_TRUE(clamped);
  EXPECT_NEAR(force.norm(), 5.0, 1e-12);
  EXPECT_NEAR(force.x(), force.y(), 1e-12);
  EXPECT_LT(force.x(), 0.0);

  vbd::updateAvbdHalfSpaceFrictionTangentPair(
      rowX, rowY, Vec3(1.0, 1.0, 0.0), options);
  EXPECT_NEAR(std::hypot(rowX.state.lambda, rowY.state.lambda), 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionTangentPairSwitchesToDynamicSlipDirection)
{
  vbd::AvbdHalfSpaceFrictionRow rowX;
  rowX.vertex = 0;
  rowX.stepStartPosition = Vec3::Zero();
  rowX.axis = Vec3::UnitX();
  rowX.state.stiffness = 10.0;
  rowX.state.lambda = -5.0;
  rowX.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::AvbdHalfSpaceFrictionRow rowY;
  rowY.vertex = 0;
  rowY.stepStartPosition = Vec3::Zero();
  rowY.axis = Vec3::UnitY();
  rowY.state.stiffness = 20.0;
  rowY.state.lambda = 0.0;
  rowY.bounds = vbd::avbdFrictionTangentBounds(5.0);

  vbd::AvbdHalfSpaceFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_FALSE(vbd::avbdFrictionPreviousDualInsideCone(rowX, rowY));
  const Eigen::Vector2d force = vbd::avbdHalfSpaceFrictionTangentPairForce(
      Vec3(0.0, 2.0, 0.0), rowX, rowY, options);
  EXPECT_NEAR(force.x(), 0.0, 1e-12);
  EXPECT_NEAR(force.y(), -5.0, 1e-12);

  vbd::updateAvbdHalfSpaceFrictionTangentPair(
      rowX, rowY, Vec3(0.0, 2.0, 0.0), options);
  EXPECT_NEAR(rowX.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(rowY.state.lambda, -5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 20.0);
}

//==============================================================================
TEST(VbdContact, AvbdBoxContactFeatureCodeSeparatesBoxManifolds)
{
  const Vec3 halfExtents(1.0, 2.0, 3.0);

  const std::uint64_t positiveXFace
      = vbd::avbdBoxContactFeatureCode(Vec3(1.2, 0.0, 0.0), halfExtents);
  const std::uint64_t positiveXFaceInside
      = vbd::avbdBoxContactFeatureCode(Vec3(0.95, 0.0, 0.0), halfExtents);
  const std::uint64_t positiveYFace
      = vbd::avbdBoxContactFeatureCode(Vec3(0.0, 2.2, 0.0), halfExtents);
  const std::uint64_t positiveXPositiveYEdge
      = vbd::avbdBoxContactFeatureCode(Vec3(1.2, 2.2, 0.0), halfExtents);
  const std::uint64_t positiveCorner
      = vbd::avbdBoxContactFeatureCode(Vec3(1.2, 2.2, 3.2), halfExtents);

  EXPECT_EQ(positiveXFaceInside, positiveXFace);
  EXPECT_NE(positiveYFace, positiveXFace);
  EXPECT_NE(positiveXPositiveYEdge, positiveXFace);
  EXPECT_NE(positiveXPositiveYEdge, positiveYFace);
  EXPECT_NE(positiveCorner, positiveXPositiveYEdge);

  EXPECT_NE(
      vbd::packAvbdBoxContactFeatureId(0, positiveXFace),
      vbd::packAvbdBoxContactFeatureId(1, positiveXFace));
  EXPECT_NE(
      vbd::packAvbdBoxContactFeatureId(0, positiveXFace),
      vbd::packAvbdBoxContactFeatureId(0, positiveYFace));

  EXPECT_EQ(
      vbd::avbdBoxContactFeatureKind(positiveXFace),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdBoxContactFeatureKind(positiveXPositiveYEdge),
      vbd::AvbdContactFeatureKind::Edge);
  EXPECT_EQ(
      vbd::avbdBoxContactFeatureKind(positiveCorner),
      vbd::AvbdContactFeatureKind::Vertex);
}

//==============================================================================
TEST(VbdContact, AvbdCylinderContactFeatureCodeSeparatesManifolds)
{
  const double radius = 1.0;
  const double halfHeight = 2.0;

  const std::uint64_t side = vbd::avbdCylinderContactFeatureCode(
      Vec3(1.2, 0.0, 0.0), radius, halfHeight);
  const std::uint64_t sideInside = vbd::avbdCylinderContactFeatureCode(
      Vec3(0.95, 0.0, 0.0), radius, halfHeight);
  const std::uint64_t topCap = vbd::avbdCylinderContactFeatureCode(
      Vec3(0.0, 0.0, 2.2), radius, halfHeight);
  const std::uint64_t bottomCap = vbd::avbdCylinderContactFeatureCode(
      Vec3(0.0, 0.0, -2.2), radius, halfHeight);
  const std::uint64_t topRim = vbd::avbdCylinderContactFeatureCode(
      Vec3(1.2, 0.0, 2.2), radius, halfHeight);
  const std::uint64_t bottomRim = vbd::avbdCylinderContactFeatureCode(
      Vec3(1.2, 0.0, -2.2), radius, halfHeight);

  EXPECT_EQ(sideInside, side);
  EXPECT_NE(topCap, side);
  EXPECT_NE(bottomCap, topCap);
  EXPECT_NE(topRim, topCap);
  EXPECT_NE(bottomRim, topRim);

  EXPECT_NE(
      vbd::packAvbdCylinderContactFeatureId(0, side),
      vbd::packAvbdBoxContactFeatureId(0, side));
  EXPECT_NE(
      vbd::packAvbdCylinderContactFeatureId(0, side),
      vbd::packAvbdCylinderContactFeatureId(1, side));

  EXPECT_EQ(
      vbd::avbdCylinderContactFeatureKind(side),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdCylinderContactFeatureKind(topCap),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdCylinderContactFeatureKind(topRim),
      vbd::AvbdContactFeatureKind::Edge);
}

//==============================================================================
TEST(VbdContact, AvbdCapsuleContactFeatureCodeSeparatesManifolds)
{
  const double halfHeight = 2.0;

  const std::uint64_t side
      = vbd::avbdCapsuleContactFeatureCode(Vec3(1.2, 0.0, 0.0), halfHeight);
  const std::uint64_t sideInside
      = vbd::avbdCapsuleContactFeatureCode(Vec3(0.95, 0.0, 0.0), halfHeight);
  const std::uint64_t topCap
      = vbd::avbdCapsuleContactFeatureCode(Vec3(0.0, 0.0, 2.2), halfHeight);
  const std::uint64_t bottomCap
      = vbd::avbdCapsuleContactFeatureCode(Vec3(0.0, 0.0, -2.2), halfHeight);

  EXPECT_EQ(sideInside, side);
  EXPECT_NE(topCap, side);
  EXPECT_NE(bottomCap, topCap);

  EXPECT_NE(
      vbd::packAvbdCapsuleContactFeatureId(0, side),
      vbd::packAvbdCylinderContactFeatureId(0, side));
  EXPECT_NE(
      vbd::packAvbdCapsuleContactFeatureId(0, side),
      vbd::packAvbdCapsuleContactFeatureId(1, side));

  EXPECT_EQ(
      vbd::avbdCapsuleContactFeatureKind(side),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdCapsuleContactFeatureKind(topCap),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(VbdContact, AvbdContactFeatureIdsRoundTripKindAndIndex)
{
  const std::uint64_t vertex
      = vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Vertex, 42);
  const std::uint64_t edge
      = vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Edge, 42);
  const std::uint64_t face
      = vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 7);

  EXPECT_EQ(
      vbd::avbdContactFeatureKind(vertex), vbd::AvbdContactFeatureKind::Vertex);
  EXPECT_EQ(vbd::avbdContactFeatureLocalIndex(vertex), 42u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(edge), vbd::AvbdContactFeatureKind::Edge);
  EXPECT_EQ(vbd::avbdContactFeatureLocalIndex(edge), 42u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(face), vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(vbd::avbdContactFeatureLocalIndex(face), 7u);
  EXPECT_NE(vertex, edge);
}

//==============================================================================
TEST(VbdContact, AvbdContactManifoldRowKeyCanonicalizesBodyOrder)
{
  const vbd::AvbdContactEndpointId first{
      17,
      vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Vertex, 3)};
  const vbd::AvbdContactEndpointId second{
      4, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 9)};

  const vbd::AvbdScalarRowKey forward = vbd::makeAvbdContactManifoldRowKey(
      vbd::AvbdScalarRowRole::ContactNormal,
      first,
      second,
      /*row=*/2,
      /*axis=*/0);
  const vbd::AvbdScalarRowKey reverse = vbd::makeAvbdContactManifoldRowKey(
      vbd::AvbdScalarRowRole::ContactNormal,
      second,
      first,
      /*row=*/2,
      /*axis=*/0);
  const vbd::AvbdScalarRowKey tangent = vbd::makeAvbdContactManifoldRowKey(
      vbd::AvbdScalarRowRole::FrictionTangent,
      first,
      second,
      /*row=*/2,
      /*axis=*/1);

  EXPECT_EQ(forward, reverse);
  EXPECT_EQ(forward.objectA, second.object);
  EXPECT_EQ(forward.featureA, second.feature);
  EXPECT_EQ(forward.objectB, first.object);
  EXPECT_EQ(forward.featureB, first.feature);
  EXPECT_NE(forward, tangent);
}

//==============================================================================
TEST(VbdContact, AvbdContactDescriptorsUseCanonicalKeysAndBounds)
{
  const vbd::AvbdContactEndpointId first{
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Edge, 4)};
  const vbd::AvbdContactEndpointId second{
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 2)};

  const vbd::AvbdScalarRowDescriptor normal
      = vbd::makeAvbdContactNormalRowDescriptor(
          first, second, /*startStiffness=*/80.0, /*maxStiffness=*/400.0);
  const vbd::AvbdScalarRowDescriptor reverseNormal
      = vbd::makeAvbdContactNormalRowDescriptor(
          second, first, /*startStiffness=*/80.0, /*maxStiffness=*/400.0);
  const vbd::AvbdScalarRowDescriptor firstTangent
      = vbd::makeAvbdContactFrictionRowDescriptor(
          first,
          second,
          /*axis=*/0,
          /*forceLimit=*/6.0,
          /*startStiffness=*/80.0,
          /*maxStiffness=*/400.0);
  const vbd::AvbdScalarRowDescriptor secondTangent
      = vbd::makeAvbdContactFrictionRowDescriptor(
          first,
          second,
          /*axis=*/1,
          /*forceLimit=*/6.0,
          /*startStiffness=*/80.0,
          /*maxStiffness=*/400.0);

  EXPECT_EQ(normal.key, reverseNormal.key);
  EXPECT_EQ(normal.key.role, vbd::AvbdScalarRowRole::ContactNormal);
  EXPECT_EQ(normal.key.objectA, second.object);
  EXPECT_EQ(normal.key.featureA, second.feature);
  EXPECT_EQ(normal.kind, vbd::AvbdScalarRowKind::HardConstraint);
  EXPECT_DOUBLE_EQ(normal.bounds.lower, 0.0);
  EXPECT_TRUE(std::isinf(normal.bounds.upper));
  EXPECT_DOUBLE_EQ(normal.startStiffness, 80.0);
  EXPECT_DOUBLE_EQ(normal.maxStiffness, 400.0);

  EXPECT_EQ(firstTangent.key.role, vbd::AvbdScalarRowRole::FrictionTangent);
  EXPECT_EQ(firstTangent.key.objectA, second.object);
  EXPECT_EQ(firstTangent.key.axis, 0u);
  EXPECT_EQ(secondTangent.key.axis, 1u);
  EXPECT_DOUBLE_EQ(firstTangent.bounds.lower, -6.0);
  EXPECT_DOUBLE_EQ(firstTangent.bounds.upper, 6.0);
  EXPECT_NE(normal.key, firstTangent.key);
  EXPECT_NE(firstTangent.key, secondTangent.key);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionDualProjectionPreservesWorldImpulse)
{
  const Eigen::Vector2d projected = vbd::projectAvbdFrictionDualToTangentPair(
      3.0, 4.0, Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitY(), -Vec3::UnitX());

  EXPECT_NEAR(projected.x(), 4.0, 1e-12);
  EXPECT_NEAR(projected.y(), -3.0, 1e-12);
  EXPECT_NEAR(projected.norm(), 5.0, 1e-12);
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
TEST(VbdContact, AvbdGroundContactUpdatesDualStateDuringSolve)
{
  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeListAllocator();
  const auto allocationsBefore = allocator.getAllocationCount();

  std::vector<Vec3> positions = {Vec3(0.0, -0.1, 0.0)};
  std::vector<double> masses = {1.0};
  std::vector<std::uint8_t> fixed = {0u};
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(1, springs);
  const auto adjacency = vbd::SpringAdjacency::build(1, springs);
  const std::vector<Vec3> inertialTargets = positions;

  vbd::ContactPlane ground;
  ground.normal = Vec3(0.0, 1.0, 0.0);
  ground.offset = 0.0;

  vbd::AvbdHalfSpaceContactRow contact;
  contact.vertex = 0;
  contact.plane = ground;
  contact.state.stiffness = 50.0;
  contact.state.lambda = 0.0;
  contact.previousConstraintValue = 0.0;
  AllocatorVector<vbd::AvbdHalfSpaceContactRow> contacts(
      common::StlAllocator<vbd::AvbdHalfSpaceContactRow>{allocator});
  contacts.push_back(contact);
  EXPECT_GT(allocator.getAllocationCount(), allocationsBefore);
  EXPECT_EQ(
      contacts.get_allocator(),
      common::StlAllocator<vbd::AvbdHalfSpaceContactRow>{allocator});

  vbd::BlockDescentOptions options;
  options.iterations = 4;
  vbd::AvbdHalfSpaceContactOptions avbdOptions;
  avbdOptions.alpha = 0.0;
  avbdOptions.beta = 100.0;

  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpringAvbdGround(
      positions,
      masses,
      fixed,
      inertialTargets,
      springs,
      0.0,
      0.1,
      contacts,
      coloring,
      adjacency,
      options,
      avbdOptions);

  EXPECT_EQ(stats.iterations, 4u);
  EXPECT_GT(positions[0].y(), -0.1);
  EXPECT_LT(positions[0].y(), 0.0);
  EXPECT_GT(contacts[0].state.lambda, 0.0);
  EXPECT_GT(contacts[0].state.stiffness, 50.0);
}

//==============================================================================
TEST(VbdContact, AvbdFrictionTangentRowsReduceTangentialMotionDuringSolve)
{
  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeListAllocator();

  std::vector<Vec3> positions = {Vec3(0.0, -0.1, 0.0)};
  std::vector<double> masses = {1.0};
  std::vector<std::uint8_t> fixed = {0u};
  const std::vector<Vec3> inertialTargets = {Vec3(1.0, -0.1, 0.0)};
  const std::vector<vbd::SpringElement> springs;
  const auto coloring = vbd::colorSprings(1, springs);
  const auto adjacency = vbd::SpringAdjacency::build(1, springs);

  AllocatorVector<vbd::AvbdHalfSpaceContactRow> contacts(
      common::StlAllocator<vbd::AvbdHalfSpaceContactRow>{allocator});
  AllocatorVector<vbd::AvbdPointAttachmentRow> attachments(
      common::StlAllocator<vbd::AvbdPointAttachmentRow>{allocator});
  AllocatorVector<vbd::AvbdSpringFiniteStiffnessRow> springRows(
      common::StlAllocator<vbd::AvbdSpringFiniteStiffnessRow>{allocator});
  AllocatorVector<vbd::AvbdHalfSpaceFrictionRow> frictionRows(
      common::StlAllocator<vbd::AvbdHalfSpaceFrictionRow>{allocator});
  frictionRows.emplace_back();
  EXPECT_EQ(
      frictionRows.get_allocator(),
      common::StlAllocator<vbd::AvbdHalfSpaceFrictionRow>{allocator});
  frictionRows[0].vertex = 0;
  frictionRows[0].stepStartPosition = positions[0];
  frictionRows[0].axis = Vec3::UnitX();
  frictionRows[0].state.stiffness = 200.0;
  frictionRows[0].state.lambda = 0.0;
  frictionRows[0].bounds = vbd::avbdFrictionTangentBounds(100.0);

  vbd::BlockDescentOptions options;
  options.iterations = 4;
  vbd::AvbdHalfSpaceContactOptions contactOptions;
  vbd::AvbdPointAttachmentOptions attachmentOptions;
  vbd::AvbdSpringFiniteStiffnessOptions springOptions;
  vbd::AvbdHalfSpaceFrictionOptions frictionOptions;
  frictionOptions.alpha = 0.0;
  frictionOptions.beta = 1.0;

  const vbd::BlockDescentStats stats = vbd::blockDescentMassSpringAvbdRows(
      positions,
      masses,
      fixed,
      inertialTargets,
      springs,
      0.0,
      0.1,
      contacts,
      attachments,
      springRows,
      coloring,
      adjacency,
      options,
      contactOptions,
      attachmentOptions,
      springOptions,
      &frictionRows,
      &frictionOptions);

  EXPECT_EQ(stats.iterations, 4u);
  EXPECT_GT(positions[0].x(), 0.05);
  EXPECT_LT(positions[0].x(), 0.6);
  EXPECT_LT(frictionRows[0].state.lambda, 0.0);
  EXPECT_GT(frictionRows[0].state.stiffness, 200.0);
}

//==============================================================================
// A small pinned spring net sagging under gravity onto a ground plane stays
// above the plane.
TEST(VbdContact, SpringNetRestsAboveGround)
{
  constexpr int kSide = 5;
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<vbd::SpringElement> springs;
  const auto index = [](int r, int c) {
    return static_cast<std::uint32_t>(r * kSide + c);
  };
  for (int r = 0; r < kSide; ++r) {
    for (int c = 0; c < kSide; ++c) {
      positions.emplace_back(c * 0.2, 0.4, r * 0.2);
      masses.push_back(1.0);
      fixed.push_back((r == 0 && (c == 0 || c == kSide - 1)) ? 1u : 0u);
    }
  }
  for (int r = 0; r < kSide; ++r) {
    for (int c = 0; c < kSide; ++c) {
      if (c + 1 < kSide) {
        springs.push_back({index(r, c), index(r, c + 1), 0.2});
      }
      if (r + 1 < kSide) {
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
