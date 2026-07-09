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
#include <dart/collision/native/narrow_phase/PrimitiveCcd.hpp>

#include <gtest/gtest.h>

#include <vector>

using namespace dart::collision::native;

namespace {

// A triangle in the z = 0 plane whose interior contains the origin's
// projection.
const Eigen::Vector3d kA(-1.0, -1.0, 0.0);
const Eigen::Vector3d kB(1.0, -1.0, 0.0);
const Eigen::Vector3d kC(0.0, 1.0, 0.0);

} // namespace

//==============================================================================
// Point-triangle ACCD
//==============================================================================

TEST(PointTriangleCcd, DirectDropHit)
{
  // Point falls straight through the static triangle; crosses z=0 at t=0.5.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_EQ(result.status, CcdPrimitiveStatus::Hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LE(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.timeOfImpact, 0.5, 1e-2);
}

TEST(PointTriangleCcd, MissBesideTriangle)
{
  // Point crosses the triangle's plane far outside the triangle.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(5, 0, 1),
      Eigen::Vector3d(5, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.isHit());
  EXPECT_EQ(result.status, CcdPrimitiveStatus::Miss);
}

TEST(PointTriangleCcd, IterationExhaustionIsIndeterminate)
{
  CcdOption option;
  option.maxIterations = 1;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.isHit());
  EXPECT_EQ(result.status, CcdPrimitiveStatus::Indeterminate);
}

TEST(PointTriangleCcd, BothMoving)
{
  // Point descends z:1->0 while the triangle rises z:0->0.5; meet at t=2/3.
  CcdOption option;
  CcdPrimitiveResult result;

  const Eigen::Vector3d up(0, 0, 0.5);
  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, 0),
      kA,
      kA + up,
      kB,
      kB + up,
      kC,
      kC + up,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_LE(result.timeOfImpact, 2.0 / 3.0 + 1e-9);
  EXPECT_NEAR(result.timeOfImpact, 2.0 / 3.0, 1e-2);
}

TEST(PointTriangleCcd, AlreadyWithinSeparation)
{
  // Point starts inside the minimum-separation band -> immediate contact.
  CcdOption option;
  option.minSeparation = 0.1;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 0.05),
      Eigen::Vector3d(0, 0, 0.05),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.0, 1e-12);
}

TEST(PointTriangleCcd, MinimumSeparationAdvancesImpact)
{
  // With a 0.2 gap, contact is when |1 - 2t| = 0.2 -> t = 0.4.
  CcdOption option;
  option.minSeparation = 0.2;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_LE(result.timeOfImpact, 0.4 + 1e-9);
  EXPECT_NEAR(result.timeOfImpact, 0.4, 1e-2);
}

TEST(PointTriangleCcd, NoRelativeMotion)
{
  // Point and triangle translate together; clearance never changes -> no hit.
  CcdOption option;
  CcdPrimitiveResult result;

  const Eigen::Vector3d shift(0, 0, 1);
  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, 1) + shift,
      kA,
      kA + shift,
      kB,
      kB + shift,
      kC,
      kC + shift,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(PointTriangleCcd, IterationCapDoesNotFalsePositive)
{
  // A point gliding at height 0.3 over the triangle in +x never touches z = 0,
  // but the slow approach can exhaust a small iteration budget before that is
  // proven. ACCD must report no contact, not a false-positive hit, when the
  // budget runs out without converging.
  const Eigen::Vector3d p0(0, 0, 0.3);
  const Eigen::Vector3d p1(4, 0, 0.3);
  CcdPrimitiveResult result;

  // Default budget: the no-contact early-out proves no impact occurs in [0, 1].
  CcdOption option;
  EXPECT_FALSE(
      pointTriangleCcd(p0, p1, kA, kA, kB, kB, kC, kC, option, result));
  EXPECT_FALSE(result.isHit());

  // Tiny budget: the loop caps before proving anything; it must still report no
  // contact rather than overshoot into a hit.
  CcdOption capped;
  capped.maxIterations = 3;
  EXPECT_FALSE(
      pointTriangleCcd(p0, p1, kA, kA, kB, kB, kC, kC, capped, result));
  EXPECT_FALSE(result.isHit());
}

TEST(PointTriangleCcd, ContactExactlyAtEndOfStep)
{
  // The point reaches the triangle plane exactly at t = 1; the inclusive
  // interval [0, 1] means this is a hit, not a miss.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, 0),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.isHit());
  EXPECT_NEAR(result.timeOfImpact, 1.0, 1e-6);
}

TEST(PointTriangleCcd, LargeGapNearMissNotReportedAsHit)
{
  // Start far above the triangle (large initial gap) and stop just short of it
  // with a small positive gap. The contact band is absolute, so this near-miss
  // must not be reported as a hit -- a gap-relative band would inflate with the
  // large starting separation and falsely flag the positive final clearance.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcd(
      Eigen::Vector3d(0, 0, 10.0),
      Eigen::Vector3d(0, 0, 5e-4), // ends 5e-4 above the triangle: no contact
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.isHit());
}

//==============================================================================
// Edge-edge ACCD
//==============================================================================

TEST(EdgeEdgeCcd, CrossingHit)
{
  // Edge A (along x at z) descends z:0.5->-0.5 across static edge B (along y).
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcd(
      Eigen::Vector3d(-1, 0, 0.5),
      Eigen::Vector3d(-1, 0, -0.5),
      Eigen::Vector3d(1, 0, 0.5),
      Eigen::Vector3d(1, 0, -0.5),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_LE(result.timeOfImpact, 0.5);
  EXPECT_NEAR(result.timeOfImpact, 0.5, 1e-2);
}

TEST(EdgeEdgeCcd, MissApproachingButNotTouching)
{
  // Edge A descends from z=1 to z=0.6 above static edge B at z=0: never meets.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcd(
      Eigen::Vector3d(-1, 0, 1.0),
      Eigen::Vector3d(-1, 0, 0.6),
      Eigen::Vector3d(1, 0, 1.0),
      Eigen::Vector3d(1, 0, 0.6),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(EdgeEdgeCcd, MinimumSeparationAdvancesImpact)
{
  // Same crossing as CrossingHit but with a 0.2 gap: contact at |0.5-t|=0.2.
  CcdOption option;
  option.minSeparation = 0.2;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcd(
      Eigen::Vector3d(-1, 0, 0.5),
      Eigen::Vector3d(-1, 0, -0.5),
      Eigen::Vector3d(1, 0, 0.5),
      Eigen::Vector3d(1, 0, -0.5),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_LE(result.timeOfImpact, 0.3 + 1e-9);
  EXPECT_NEAR(result.timeOfImpact, 0.3, 1e-2);
}

TEST(EdgeEdgeCcd, ParallelStationaryNoHit)
{
  // Parallel, separated, no motion -> no relative motion, no contact.
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcd(
      Eigen::Vector3d(-1, 0, 1),
      Eigen::Vector3d(-1, 0, 1),
      Eigen::Vector3d(1, 0, 1),
      Eigen::Vector3d(1, 0, 1),
      Eigen::Vector3d(-1, 0, 0),
      Eigen::Vector3d(-1, 0, 0),
      Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(1, 0, 0),
      option,
      result);

  EXPECT_FALSE(hit);
}

//==============================================================================
// Exact coplanarity-cubic solver (validation path)
//==============================================================================

TEST(PointTriangleCcdExact, DirectDropRootAtHalf)
{
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcdExact(
      Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(0, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.5, 1e-6);
}

TEST(PointTriangleCcdExact, MissBesideTriangle)
{
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcdExact(
      Eigen::Vector3d(5, 0, 1),
      Eigen::Vector3d(5, 0, -1),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_FALSE(hit);
}

TEST(PointTriangleCcdExact, CoplanarInteriorCrossing)
{
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = pointTriangleCcdExact(
      Eigen::Vector3d(-2, 0, 0),
      Eigen::Vector3d(2, 0, 0),
      kA,
      kA,
      kB,
      kB,
      kC,
      kC,
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
  EXPECT_NEAR(result.timeOfImpact, 0.375, 0.05);
}

TEST(EdgeEdgeCcdExact, CrossingRootAtHalf)
{
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcdExact(
      Eigen::Vector3d(-1, 0, 0.5),
      Eigen::Vector3d(-1, 0, -0.5),
      Eigen::Vector3d(1, 0, 0.5),
      Eigen::Vector3d(1, 0, -0.5),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_NEAR(result.timeOfImpact, 0.5, 1e-6);
}

TEST(EdgeEdgeCcdExact, CoplanarInteriorCrossing)
{
  // Both edges stay in the z = 0 plane, so the coplanarity cubic is identically
  // zero and yields no interior roots. A static horizontal edge (x in [-1, 1])
  // is swept across by a vertical edge whose x glides -2 -> 2; the two overlap
  // once the vertical edge enters the horizontal edge's span at t = 0.25 -- an
  // interior first contact the exact path must still detect (not just t = 0/1).
  CcdOption option;
  CcdPrimitiveResult result;

  const bool hit = edgeEdgeCcdExact(
      Eigen::Vector3d(-1, 0, 0), // edge 1 (a): static
      Eigen::Vector3d(-1, 0, 0),
      Eigen::Vector3d(1, 0, 0), // edge 1 (b): static
      Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(-2, -1, 0), // edge 2 (c): sweeps -2 -> 2 in x
      Eigen::Vector3d(2, -1, 0),
      Eigen::Vector3d(-2, 1, 0), // edge 2 (d): sweeps -2 -> 2 in x
      Eigen::Vector3d(2, 1, 0),
      option,
      result);

  EXPECT_TRUE(hit);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
  EXPECT_NEAR(result.timeOfImpact, 0.25, 0.05);
}

//==============================================================================
// Conservativeness: ACCD must never overshoot the exact time of impact
//==============================================================================

TEST(PrimitiveCcdConservativeness, PointTriangleNeverOvershoots)
{
  CcdOption option;

  struct Case
  {
    Eigen::Vector3d p0, p1, aOff, bOff, cOff;
  };
  const std::vector<Case> cases = {
      {{0, 0, 1}, {0, 0, -1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
      {{0.3, -0.2, 1.5}, {0.1, 0.2, -0.5}, {0, 0, 0.2}, {0, 0, 0.1}, {0, 0, 0}},
      {{-0.4, 0.3, 2.0},
       {0.2, -0.1, -1.0},
       {0.1, 0, 0},
       {0, 0.1, 0},
       {0, 0, 0.1}},
  };

  for (const auto& tc : cases) {
    CcdPrimitiveResult accd;
    CcdPrimitiveResult exact;
    const bool accdHit = pointTriangleCcd(
        tc.p0,
        tc.p1,
        kA,
        kA + tc.aOff,
        kB,
        kB + tc.bOff,
        kC,
        kC + tc.cOff,
        option,
        accd);
    const bool exactHit = pointTriangleCcdExact(
        tc.p0,
        tc.p1,
        kA,
        kA + tc.aOff,
        kB,
        kB + tc.bOff,
        kC,
        kC + tc.cOff,
        option,
        exact);

    if (exactHit) {
      EXPECT_TRUE(accdHit);
      EXPECT_LE(accd.timeOfImpact, exact.timeOfImpact + 1e-6);
    }
  }
}

TEST(PrimitiveCcdConservativeness, EdgeEdgeNeverOvershoots)
{
  CcdOption option;

  CcdPrimitiveResult accd;
  CcdPrimitiveResult exact;
  const bool accdHit = edgeEdgeCcd(
      Eigen::Vector3d(-1, 0, 0.5),
      Eigen::Vector3d(-1, 0, -0.5),
      Eigen::Vector3d(1, 0, 0.5),
      Eigen::Vector3d(1, 0, -0.5),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      accd);
  const bool exactHit = edgeEdgeCcdExact(
      Eigen::Vector3d(-1, 0, 0.5),
      Eigen::Vector3d(-1, 0, -0.5),
      Eigen::Vector3d(1, 0, 0.5),
      Eigen::Vector3d(1, 0, -0.5),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, -1, 0),
      Eigen::Vector3d(0, 1, 0),
      Eigen::Vector3d(0, 1, 0),
      option,
      exact);

  ASSERT_TRUE(exactHit);
  ASSERT_TRUE(accdHit);
  EXPECT_LE(accd.timeOfImpact, exact.timeOfImpact + 1e-6);
}

//==============================================================================
// Determinism
//==============================================================================

TEST(PrimitiveCcd, Determinism)
{
  CcdOption option;

  double first = -1.0;
  for (int i = 0; i < 100; ++i) {
    CcdPrimitiveResult result;
    const bool hit = pointTriangleCcd(
        Eigen::Vector3d(0.12, -0.07, 1.0),
        Eigen::Vector3d(-0.05, 0.09, -1.0),
        kA,
        kA,
        kB,
        kB,
        kC,
        kC,
        option,
        result);
    ASSERT_TRUE(hit);
    if (i == 0) {
      first = result.timeOfImpact;
    } else {
      EXPECT_EQ(result.timeOfImpact, first);
    }
  }
}
