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

#include "../../examples/demos/scenes/FbfAuthorMasonryArchSpec.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>

#include <cmath>
#include <cstdint>

namespace {

void appendSignedIntegerToFnv1a(std::uint64_t& hash, std::int64_t value)
{
  constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
  const std::uint64_t bits = static_cast<std::uint64_t>(value);
  for (std::size_t byte = 0u; byte < sizeof(bits); ++byte) {
    hash ^= (bits >> (8u * byte)) & 0xffu;
    hash *= kFnvPrime;
  }
}

std::uint64_t sourceObjVertexRecordHash(
    const fbf_author_masonry_arch::Vertices& vertices)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto& vertex : vertices) {
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      appendSignedIntegerToFnv1a(
          hash, static_cast<std::int64_t>(std::llround(vertex[axis] * 1.0e6)));
    }
  }
  return hash;
}

TEST(FbfAuthorMasonryArchSpec, SourceConstructionIsPinned)
{
  using namespace fbf_author_masonry_arch;

  EXPECT_STREQ(kAuthorCommit, "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0");
  EXPECT_STREQ(kAuthorTree, "ffcdafb61adeda2239c8366d054b548b50d26685");
  EXPECT_STREQ(kAuthorRunBlob, "35a052d7ef0975e7c828c9678d163054dfbb3ef2");
  EXPECT_STREQ(
      kAuthorRunSha256,
      "7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73");
  EXPECT_STREQ(kAuthorMeshTree, "2552017df4061c49f7df064d847a4268a6e02d1a");
  EXPECT_STREQ(
      kAuthorMeshTreeSha256,
      "a3f4e35073a2f4e74837fff277cd923f104b6af57f2cf995cf7524fe498e483d");
  EXPECT_STREQ(kAuthorMeshDirectory, "meshes/arch/num_stones=25");
  const std::string specSourceSha256(kSpecSourceSha256);
  EXPECT_EQ(specSourceSha256.size(), 64u);
  EXPECT_TRUE(std::all_of(
      specSourceSha256.begin(), specSourceSha256.end(), [](char value) {
        return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
      }));

  EXPECT_EQ(kStoneCount, 25u);
  EXPECT_EQ(kFixedSpringerCount, 2u);
  EXPECT_TRUE(kFixSpringers);
  EXPECT_EQ(kCubeCount, 3u);
  EXPECT_TRUE(kCubesInitiallyKinematic);
  EXPECT_STREQ(kGroundShape, "plane");
  EXPECT_FALSE(kGroundMobile);
  EXPECT_DOUBLE_EQ(kFriction, 0.8);
  EXPECT_DOUBLE_EQ(kStoneDensity, 2000.0);
  EXPECT_DOUBLE_EQ(kScale, 1.0);
  EXPECT_DOUBLE_EQ(kContactGap, 0.005);
  EXPECT_DOUBLE_EQ(kShapeStiffness, 1.0e4);
  EXPECT_DOUBLE_EQ(kShapeDamping, 1.0e3);
  EXPECT_EQ(triangles().size(), 12u);

  const auto stones = makeStoneSpecs();
  ASSERT_EQ(stones.size(), kStoneCount);

  // These FNV-1a values cover the 24 signed integer micro-units in each
  // pinned source OBJ's eight vertex records after the author's y/z swap.
  // They bind every one of the 25 files, rather than only sample stones.
  constexpr std::array<std::uint64_t, kStoneCount> kExpectedVertexHashes{{
      0xf8d666193755aa0dULL, 0x85b3f54b9ab520b1ULL, 0x89b367cb28c68931ULL,
      0x6dac7f9d7239b231ULL, 0xf77703f9b949ef3dULL, 0xdbb9a505c1bbd945ULL,
      0x3a5505348c2bf2ddULL, 0x0375babad7f7f235ULL, 0x9895f50a4446d1a1ULL,
      0x7025700a603449f9ULL, 0x487a2f47f832b615ULL, 0x414cc8454d4c808dULL,
      0x243df8c881cfa9fdULL, 0xa3b32dbea05140c9ULL, 0xbed0aa58dcaeefddULL,
      0x4878440fd68c5489ULL, 0xab7103ad8fc24e4dULL, 0x1598c7368bd34c95ULL,
      0xd54d1eca8943b2fdULL, 0xddbec17d34c68c3dULL, 0x1dd495ce7eef336dULL,
      0xebb528344bee6125ULL, 0x5233123896663f95ULL, 0x3ac3e92185d42951ULL,
      0x7be33a3a46f68e41ULL,
  }};

  std::size_t fixedStoneCount = 0u;
  for (std::size_t index = 0u; index < stones.size(); ++index) {
    const auto& stone = stones[index];
    EXPECT_EQ(stone.index, index);
    EXPECT_EQ(
        stone.name,
        "stone-" + (index < 9u ? std::string("0") : std::string())
            + std::to_string(index + 1u));
    EXPECT_EQ(
        sourceObjVertexRecordHash(stone.vertices),
        kExpectedVertexHashes[index]);
    EXPECT_DOUBLE_EQ(stone.density, kStoneDensity);
    EXPECT_DOUBLE_EQ(stone.friction, kFriction);
    EXPECT_GT(stone.volume, 0.0);
    EXPECT_DOUBLE_EQ(stone.mass, kStoneDensity * stone.volume);
    EXPECT_TRUE(stone.transform.linear().isIdentity(0.0));
    EXPECT_TRUE(stone.transform.translation().isApprox(
        vertexMean(stone.vertices), 1e-14));
    EXPECT_TRUE(stone.localCenterOfMass.allFinite());
    EXPECT_TRUE(stone.moment.allFinite());
    EXPECT_TRUE(stone.moment.isApprox(stone.moment.transpose(), 1e-10));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        stone.moment);
    ASSERT_EQ(eigenSolver.info(), Eigen::Success);
    EXPECT_GT(eigenSolver.eigenvalues().minCoeff(), 0.0);
    if (!stone.mobile)
      ++fixedStoneCount;

    const Eigen::Vector3d centerOfMass
        = stone.transform * stone.localCenterOfMass;
    for (const auto& triangle : triangles()) {
      const auto& a = stone.vertices[triangle[0]];
      const auto& b = stone.vertices[triangle[1]];
      const auto& c = stone.vertices[triangle[2]];
      const Eigen::Vector3d normal = (b - a).cross(c - a);
      ASSERT_GT(normal.norm(), 1e-10);
      EXPECT_LE(normal.dot(centerOfMass - a), 1e-8);
    }
  }
  EXPECT_EQ(fixedStoneCount, kFixedSpringerCount);
  EXPECT_FALSE(stones.front().mobile);
  EXPECT_TRUE(stones[1].mobile);
  EXPECT_TRUE(stones[kStoneCount - 2u].mobile);
  EXPECT_FALSE(stones.back().mobile);

  EXPECT_TRUE(stones.front().vertices.front().isApprox(
      Eigen::Vector3d(-26.315009, -5.0, 0.1), 0.0));
  EXPECT_TRUE(stones[12].vertices.front().isApprox(
      Eigen::Vector3d(-1.694454, -3.510962, 58.582120), 0.0));
  EXPECT_TRUE(stones.back().vertices.back().isApprox(
      Eigen::Vector3d(36.565382, 5.0, 0.1), 0.0));
  EXPECT_TRUE(stones.front().transform.translation().isApprox(
      Eigen::Vector3d(-30.703668, 0.0, 3.36949725), 1e-14));
  EXPECT_TRUE(stones[12].transform.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 61.9277475), 1e-14));
  EXPECT_NEAR(stones.front().volume, 670.27053898891984, 1e-9);
  EXPECT_NEAR(stones.front().mass, 1340541.0779778396, 1e-6);
  EXPECT_NEAR(stones.front().moment(0, 0), 16233093.511982396, 1e-5);
  EXPECT_NEAR(stones.front().moment(0, 2), 143921.46173566594, 1e-6);
  EXPECT_NEAR(stones[12].volume, 259.28234604317919, 1e-9);
  EXPECT_NEAR(stones[12].mass, 518564.69208635838, 1e-6);
  EXPECT_NEAR(stones[12].moment(1, 1), 3350674.6906006681, 1e-5);
  EXPECT_DOUBLE_EQ(archTopZ(stones), kSourceObjRecordArchTopZ);
  EXPECT_DOUBLE_EQ(kSourceRuntimeArchTopZFloat32, 65.27337646484375);
  EXPECT_NE(kSourceRuntimeArchTopZFloat32, kSourceObjRecordArchTopZ);

  const auto cubes = makeCubeSpecs();
  ASSERT_EQ(cubes.size(), kCubeCount);
  constexpr std::array<double, kCubeCount> kExpectedCubeX{{-4.5, 0.0, 4.5}};
  for (std::size_t index = 0u; index < cubes.size(); ++index) {
    const auto& cube = cubes[index];
    EXPECT_EQ(cube.name, "cube_" + std::to_string(index));
    EXPECT_TRUE(cube.size.isApprox(Eigen::Vector3d::Constant(3.0), 0.0));
    EXPECT_DOUBLE_EQ(cube.transform.translation().x(), kExpectedCubeX[index]);
    EXPECT_DOUBLE_EQ(cube.transform.translation().y(), 0.0);
    EXPECT_DOUBLE_EQ(cube.transform.translation().z(), 75.27337646484375);
    EXPECT_DOUBLE_EQ(cube.density, 2000.0);
    EXPECT_DOUBLE_EQ(cube.mass, 54000.0);
    EXPECT_TRUE(
        cube.moment.isApprox(Eigen::Matrix3d::Identity() * 81000.0, 0.0));
    EXPECT_DOUBLE_EQ(cube.friction, kFriction);
    EXPECT_FALSE(cube.initiallyMobile);
  }
}

TEST(FbfAuthorMasonryArchSpec, ContractIsConstructionOnly)
{
  using namespace fbf_author_masonry_arch;

  EXPECT_STREQ(kContractKind, "configuration_only");
  EXPECT_DOUBLE_EQ(kDisplayTimeStep, 1.0 / 60.0);
  EXPECT_EQ(kSubstepsPerFrame, 4u);
  EXPECT_DOUBLE_EQ(kRuntimeTimeStep, 1.0 / 240.0);
  EXPECT_EQ(kDefaultFrameCount, 400u);
  EXPECT_EQ(kDropFrame, 400u);
  EXPECT_EQ(kReleaseSubstep, 1600u);
  EXPECT_EQ(kDefaultTotalSubsteps, 1600u);
  EXPECT_FALSE(releaseOccursWithinFrames(kDefaultFrameCount));
  EXPECT_TRUE(releaseOccursWithinFrames(kDefaultFrameCount + 1u));
  EXPECT_FALSE(cubeIsMobileAtSubstep(1599u, kDefaultFrameCount + 1u));
  EXPECT_TRUE(cubeIsMobileAtSubstep(1600u, kDefaultFrameCount + 1u));
  EXPECT_FALSE(cubeIsMobileAtSubstep(1600u, kDefaultFrameCount));

  EXPECT_TRUE(kConfigurationPortValid);
  EXPECT_TRUE(kSourceObjNumericRecordsEquivalent);
  EXPECT_TRUE(kSourceAxisSwapImplemented);
  EXPECT_TRUE(kSourceSolverConfigurationRecorded);
  EXPECT_EQ(kSourceMaxContacts, 4096u);
  EXPECT_EQ(kSourceMaxOuterIterations, 200);
  EXPECT_DOUBLE_EQ(kSourceOuterTolerance, 1e-6);
  EXPECT_EQ(kSourceResidualCheckInterval, 1);
  EXPECT_STREQ(kSourceInnerSolver, "block_gs");
  EXPECT_EQ(kSourceInnerGaussSeidelSweeps, 30);
  EXPECT_EQ(kSourceInnerMaxIterations, 1000);
  EXPECT_DOUBLE_EQ(kSourceInnerTolerance, 1e-6);
  EXPECT_DOUBLE_EQ(kSourceGammaC, 15.5);
  EXPECT_DOUBLE_EQ(kSourceGammaMax, 1e6);
  EXPECT_TRUE(kSourceAdaptiveGamma);
  EXPECT_DOUBLE_EQ(kSourceArmijoRhoHigh, 0.9);
  EXPECT_DOUBLE_EQ(kSourceArmijoShrink, 0.7);
  EXPECT_DOUBLE_EQ(kSourceArmijoGrow, 1.0 / 0.7);
  EXPECT_DOUBLE_EQ(kSourceArmijoSkipThreshold, 1e-10);
  EXPECT_EQ(kSourceArmijoMaxBacktracks, 8);
  EXPECT_EQ(kSourcePlateauPatience, 0);
  EXPECT_DOUBLE_EQ(kSourcePlateauRelativeTolerance, 0.01);
  EXPECT_DOUBLE_EQ(kSourceWarmStartMatchRadius, 0.02);
  EXPECT_DOUBLE_EQ(kSourceWarmStartNormalCosine, 0.9);
  EXPECT_EQ(kSourceWarmStartMaxAge, 3);
  EXPECT_DOUBLE_EQ(kSourceWarmStartGammaThreshold, 1e-4);
  EXPECT_DOUBLE_EQ(kSourceWarmStartGammaCap, 1e4);
  EXPECT_DOUBLE_EQ(kSourceBaumgarteErp, 0.0);
  EXPECT_TRUE(kSourceProjectAfterCorrection);
  EXPECT_STREQ(kSourceTerminationResidual, "coulomb_rel");
  EXPECT_DOUBLE_EQ(kSourceTerminationTolerance, 1e-6);
  EXPECT_FALSE(kSourceCollisionSemanticsImplemented);
  EXPECT_FALSE(kSourceContactGapSemanticsImplemented);
  EXPECT_FALSE(kSourceSolverBackendSemanticsImplemented);
  EXPECT_FALSE(kSourceFloat32SemanticsImplemented);
  EXPECT_FALSE(kSourceRuntimeFloat32InitialPosesEquivalent);
  EXPECT_FALSE(kDynamicsExecuted);
  EXPECT_FALSE(kTrajectoryEquivalence);
  EXPECT_FALSE(kDynamicsEquivalence);
  EXPECT_FALSE(kPhysicalOutcomeEquivalence);
  EXPECT_FALSE(kFig07Parity);
  EXPECT_FALSE(kVideo07Arch25Parity);
  EXPECT_FALSE(kTimingComparability);
  EXPECT_FALSE(kPaperParity);
}

} // namespace
