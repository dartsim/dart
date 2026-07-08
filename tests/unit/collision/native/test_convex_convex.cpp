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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/collision/native/Types.hpp>
#include <dart/collision/native/narrow_phase/ConvexConvex.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d rotatedTranslated(double x, double y, double z)
{
  Eigen::Isometry3d tf = translated(x, y, z);
  tf.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  return tf;
}

} // namespace

TEST(ConvexSupportFunctions, PrimitiveSupportPointsRespectTransforms)
{
  const SphereShape sphere(0.5);
  const auto sphereSupport
      = makeSphereSupportFunction(sphere, translated(1.0, 2.0, 3.0));
  EXPECT_TRUE(sphereSupport(Eigen::Vector3d::Zero())
                  .isApprox(Eigen::Vector3d(1.5, 2.0, 3.0), 1e-12));
  EXPECT_TRUE(sphereSupport(Eigen::Vector3d::UnitY())
                  .isApprox(Eigen::Vector3d(1.0, 2.5, 3.0), 1e-12));

  const BoxShape box(Eigen::Vector3d(1.0, 2.0, 3.0));
  const auto boxSupport
      = makeBoxSupportFunction(box, rotatedTranslated(0, 0, 0));
  EXPECT_TRUE(boxSupport(Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d(2.0, 1.0, 3.0), 1e-12));

  const CapsuleShape capsule(0.25, 2.0);
  const auto capsuleSupport
      = makeCapsuleSupportFunction(capsule, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(capsuleSupport(Eigen::Vector3d::Zero())
                  .isApprox(Eigen::Vector3d(0.25, 0.0, 1.0), 1e-12));
  EXPECT_TRUE(capsuleSupport(-Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d(0.0, 0.0, -1.25), 1e-12));

  const CylinderShape cylinder(0.5, 2.0);
  const auto cylinderSupport
      = makeCylinderSupportFunction(cylinder, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(cylinderSupport(Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d(0.5, 0.0, 1.0), 1e-12));
  EXPECT_TRUE(cylinderSupport(Eigen::Vector3d::UnitY())
                  .isApprox(Eigen::Vector3d(0.0, 0.5, 1.0), 1e-12));
}

TEST(ConvexSupportFunctions, MeshAndConvexSupportUseFurthestVertex)
{
  const std::vector<Eigen::Vector3d> vertices = makeOctahedronVertices(2.0);
  const std::vector<MeshShape::Triangle> triangles{
      {0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}};
  const MeshShape mesh(vertices, triangles);
  const auto meshSupport = makeMeshSupportFunction(mesh, translated(1, 0, 0));
  EXPECT_TRUE(meshSupport(Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d(3.0, 0.0, 0.0), 1e-12));

  const ConvexShape convex(vertices);
  const auto convexSupport
      = makeConvexSupportFunction(convex, translated(0, 1, 0));
  EXPECT_TRUE(convexSupport(Eigen::Vector3d::UnitY())
                  .isApprox(Eigen::Vector3d(0.0, 3.0, 0.0), 1e-12));
}

TEST(ConvexConvex, SeparatedReturnsFalse)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_FALSE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(3.0, 0.0, 0.0),
      result,
      CollisionOption()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvex, OverlapAddsContact)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      CollisionOption()));
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_GT(result.getContact(0).depth, 0.0);
}

TEST(ConvexConvex, ConvexSphereOverlap)
{
  const ConvexShape convex(makeOctahedronVertices());
  const SphereShape sphere(0.75);

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(1.25, 0.0, 0.0),
      result,
      CollisionOption()));
  EXPECT_EQ(1u, result.numContacts());
}

TEST(ConvexConvex, PrimitiveShapesUseGenericSupportPath)
{
  const BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  const CylinderShape cylinder(0.5, 1.0);

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      box,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(0.75, 0.0, 0.0),
      result,
      CollisionOption()));
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_GT(result.getContact(0).depth, 0.0);
}

TEST(ConvexConvex, MeshShapeUsesGenericSupportPath)
{
  const std::vector<Eigen::Vector3d> vertices = makeOctahedronVertices();
  const std::vector<MeshShape::Triangle> triangles{
      {0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}};
  const MeshShape mesh(vertices, triangles);
  const BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      mesh,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0),
      result,
      CollisionOption()));
  ASSERT_EQ(1u, result.numContacts());
}

TEST(ConvexConvex, BinaryCheckDoesNotAddContacts)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvex, ZeroContactLimitReturnsFalse)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvexBatch, CollidesEachPair)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());
  const std::vector<ConvexPair> pairs{
      {&convex1,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)},
      {&convex1,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(3.0, 0.0, 0.0)}};
  std::vector<CollisionResult> results(2);

  collideConvexConvexBatch(pairs, results);

  EXPECT_EQ(1u, results[0].numContacts());
  EXPECT_EQ(0u, results[1].numContacts());
}

TEST(ConvexConvexBatch, RejectsMalformedInputs)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());
  const std::vector<ConvexPair> validPairs{
      {&convex1,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)}};

  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideConvexConvexBatch(validPairs, emptyResults),
      std::invalid_argument);

  std::vector<CollisionResult> results(1);
  const std::vector<ConvexPair> nullShapePairs{
      {nullptr,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)}};
  EXPECT_THROW(
      collideConvexConvexBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(ConvexConvexDistance, SeparatedShapesReportPositiveDistance)
{
  const BoxShape box1(Eigen::Vector3d(0.5, 0.5, 0.5));
  const BoxShape box2(Eigen::Vector3d(0.5, 0.5, 0.5));

  DistanceResult result;
  const double distance = distanceConvexConvex(
      box1,
      Eigen::Isometry3d::Identity(),
      box2,
      translated(2.0, 0.0, 0.0),
      result,
      DistanceOption::unlimited());

  EXPECT_GT(distance, 0.0);
  EXPECT_TRUE(result.isValid());
  EXPECT_NEAR(distance, result.distance, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(ConvexConvexDistance, UpperBoundPrunesSeparatedShapes)
{
  const BoxShape box1(Eigen::Vector3d(0.5, 0.5, 0.5));
  const BoxShape box2(Eigen::Vector3d(0.5, 0.5, 0.5));

  DistanceResult result;
  const double distance = distanceConvexConvex(
      box1,
      Eigen::Isometry3d::Identity(),
      box2,
      translated(3.0, 0.0, 0.0),
      result,
      DistanceOption::withUpperBound(0.25));

  EXPECT_EQ(std::numeric_limits<double>::max(), distance);
  EXPECT_FALSE(result.isValid());
}

TEST(ConvexConvexDistance, OverlappingShapesReportNegativeDistance)
{
  const SphereShape sphere(0.75);
  const BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  DistanceResult result;
  const double distance = distanceConvexConvex(
      sphere,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0),
      result,
      DistanceOption::unlimited());

  EXPECT_LT(distance, 0.0);
  EXPECT_TRUE(result.isValid());
  EXPECT_NEAR(distance, result.distance, 1e-12);
  EXPECT_TRUE(result.pointOnObject1.allFinite());
  EXPECT_TRUE(result.pointOnObject2.allFinite());
  EXPECT_TRUE(result.normal.allFinite());
}
