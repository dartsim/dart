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

#include "../../examples/demos/scenes/FbfAuthorMasonryArchDartAdapter.hpp"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

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

std::uint64_t sourceObjInventoryHash(
    const std::vector<fbf_author_masonry_arch::StoneSpec>& stones)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto& stone : stones) {
    for (const auto& vertex : stone.vertices) {
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        appendSignedIntegerToFnv1a(
            hash,
            static_cast<std::int64_t>(std::llround(vertex[axis] * 1.0e6)));
      }
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

TEST(FbfAuthorMasonryArchSpec, SourceParameterizedArch101ConstructionIsPinned)
{
  using namespace fbf_author_masonry_arch;

  const auto& scenario = sourceScenarioSpec(SourceScenario::Arch101);
  EXPECT_EQ(scenario.scenario, SourceScenario::Arch101);
  EXPECT_EQ(scenario.stoneCount, 101u);
  EXPECT_STREQ(
      scenario.authorMeshTree, "e0c209235673d2f69c3c5de7708ab1dfadec96e3");
  EXPECT_STREQ(
      scenario.authorMeshTreeSha256,
      "7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf");
  EXPECT_STREQ(scenario.authorMeshDirectory, "meshes/arch/num_stones=101");
  EXPECT_STREQ(scenario.authorStoneSelection, "--stones 101");
  EXPECT_DOUBLE_EQ(scenario.sourceObjRecordArchTopZ, 69.628159);
  EXPECT_DOUBLE_EQ(scenario.sourceRuntimeArchTopZFloat32, 69.62815856933594);
  EXPECT_DOUBLE_EQ(scenario.cubeInitialZ, 79.62815856933594);

  const auto stones = makeStoneSpecs(SourceScenario::Arch101);
  ASSERT_EQ(stones.size(), scenario.stoneCount);
  // One ordered FNV-1a digest binds all 2,424 signed micro-unit coordinates
  // from the 101 pinned source OBJ files after the author's y/z axis swap.
  EXPECT_EQ(sourceObjInventoryHash(stones), 0x528596c9206aef89ULL);

  std::size_t fixedStoneCount = 0u;
  for (std::size_t index = 0u; index < stones.size(); ++index) {
    const auto& stone = stones[index];
    EXPECT_EQ(stone.index, index);
    EXPECT_EQ(
        stone.name,
        "stone-" + (index < 9u ? std::string("0") : std::string())
            + std::to_string(index + 1u));
    EXPECT_GT(stone.volume, 0.0);
    EXPECT_GT(stone.mass, 0.0);
    EXPECT_TRUE(stone.localCenterOfMass.allFinite());
    EXPECT_TRUE(stone.moment.allFinite());
    EXPECT_TRUE(stone.moment.isApprox(stone.moment.transpose(), 1e-10));
    fixedStoneCount += stone.mobile ? 0u : 1u;
  }
  EXPECT_EQ(fixedStoneCount, kFixedSpringerCount);
  EXPECT_FALSE(stones.front().mobile);
  EXPECT_TRUE(stones[1].mobile);
  EXPECT_TRUE(stones[kStoneCount101 - 2u].mobile);
  EXPECT_FALSE(stones.back().mobile);
  EXPECT_TRUE(stones.front().vertices.front().isApprox(
      Eigen::Vector3d(-30.115009, -5.0, 0.1), 0.0));
  EXPECT_TRUE(stones[50].vertices.front().isApprox(
      Eigen::Vector3d(-0.415074, -3.500692, 62.648966), 0.0));
  EXPECT_TRUE(stones[50].vertices.back().isApprox(
      Eigen::Vector3d(0.972056, 3.500692, 69.628159), 0.0));
  EXPECT_TRUE(stones.back().vertices.back().isApprox(
      Eigen::Vector3d(40.354012, 5.0, 0.1), 0.0));
  EXPECT_DOUBLE_EQ(archTopZ(stones), scenario.sourceObjRecordArchTopZ);

  const auto cubes = makeCubeSpecs(SourceScenario::Arch101);
  ASSERT_EQ(cubes.size(), kCubeCount);
  for (const auto& cube : cubes) {
    EXPECT_DOUBLE_EQ(cube.transform.translation().z(), scenario.cubeInitialZ);
    EXPECT_FALSE(cube.initiallyMobile);
  }

  EXPECT_EQ(kDefaultFrameCount, 400u);
  EXPECT_EQ(kDefaultTotalSubsteps, 1600u);
  EXPECT_EQ(kDropFrame, kDefaultFrameCount);
  EXPECT_FALSE(releaseOccursWithinFrames(kDefaultFrameCount));
  EXPECT_FALSE(kFig08Parity);
  EXPECT_FALSE(kVideo08Arch101Parity);
  EXPECT_FALSE(kPaperParity);
}

TEST(FbfAuthorMasonryArchSpec, DartAdapterPinsInitialWorldAndReleaseAction)
{
  namespace adapter = fbf_author_masonry_arch_adapter;
  using namespace fbf_author_masonry_arch;

  EXPECT_STREQ(
      adapter::kDemoSceneId,
      "fbf_author_masonry_arch_25_crown_impact_current_source");
  EXPECT_STREQ(
      adapter::kContractSchema,
      "dart.fbf_author_masonry_arch_crown_impact_dart_adapter/v1");
  EXPECT_EQ(adapter::kEvidenceFrameCount, 500u);
  EXPECT_EQ(adapter::kEvidenceTotalSubsteps, 2000u);
  EXPECT_EQ(adapter::kReleaseActionCompletedStep, kReleaseSubstep);
  EXPECT_EQ(adapter::kReleaseActionKey, 'p');

  const auto exactWorld = adapter::createWorld(adapter::SolverLane::ExactFbf);
  const auto boxedWorld = adapter::createWorld(adapter::SolverLane::BoxedLcp);
  const auto exact = adapter::inspectAdapterContract(exactWorld);
  const auto boxed = adapter::inspectAdapterContract(boxedWorld);

  EXPECT_EQ(exact.solverLane, adapter::SolverLane::ExactFbf);
  EXPECT_EQ(boxed.solverLane, adapter::SolverLane::BoxedLcp);
  ASSERT_TRUE(exact.exactOptions.has_value());
  ASSERT_TRUE(exact.exactCrossStepOptions.has_value());
  EXPECT_TRUE(exact.exactColoredBlockGaussSeidelEnabled);
  EXPECT_TRUE(exact.exactParticipantAffinityEnabled);
  EXPECT_FALSE(boxed.exactOptions.has_value());
  EXPECT_FALSE(boxed.exactCrossStepOptions.has_value());
  EXPECT_FALSE(boxed.exactColoredBlockGaussSeidelEnabled);
  EXPECT_FALSE(boxed.exactParticipantAffinityEnabled);
  for (const auto* contract : {&exact, &boxed}) {
    EXPECT_DOUBLE_EQ(contract->timeStep, kRuntimeTimeStep);
    EXPECT_TRUE(contract->gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
    EXPECT_EQ(contract->simulationThreads, 1u);
    EXPECT_FALSE(contract->deactivationEnabled);
    EXPECT_EQ(contract->maxContacts, kSourceMaxContacts);
    EXPECT_EQ(contract->maxContactsPerPair, adapter::kDartMaxContactsPerPair);
    EXPECT_TRUE(contract->nativeFourPointPlanar);
    EXPECT_TRUE(contract->splitImpulseEnabled);
    EXPECT_FALSE(contract->cubesAreReleased);
    EXPECT_EQ(contract->stoneCount, kStoneCount);
    EXPECT_EQ(contract->mobileStoneCount, kStoneCount - kFixedSpringerCount);
    EXPECT_EQ(contract->cubeCount, kCubeCount);
  }

  auto* exactSolver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          exactWorld->getConstraintSolver());
  ASSERT_NE(exactSolver, nullptr);
  const auto options = exactSolver->getExactCoulombOptions();
  EXPECT_FALSE(options.fallbackToBoxedLcp);
  EXPECT_EQ(options.maxOuterIterations, adapter::kDartMaxOuterIterations);
  EXPECT_TRUE(options.acceptOuterMaxIterations);
  EXPECT_DOUBLE_EQ(options.tolerance, adapter::kDartResidualTolerance);
  EXPECT_DOUBLE_EQ(options.stepSizeScale, adapter::kDartStepSizeScale);
  EXPECT_DOUBLE_EQ(options.outerRelaxation, adapter::kDartOuterRelaxation);
  EXPECT_EQ(options.innerMaxSweeps, adapter::kDartInnerMaxSweeps);
  EXPECT_TRUE(options.runFixedInnerSweeps);
  EXPECT_TRUE(options.acceptInnerMaxIterations);
  EXPECT_EQ(options.innerLocalIterations, adapter::kDartInnerLocalIterations);
  const auto crossStep = exactSolver->getExactCoulombCrossStepPolicyOptions();
  EXPECT_EQ(
      crossStep.warmStartMatchMode,
      dart::constraint::ExactCoulombFbfWarmStartMatchMode::
          EitherBodyLocalFeature);
  EXPECT_DOUBLE_EQ(crossStep.warmStartNormalCosine, 0.9);
  EXPECT_FALSE(crossStep.useStrictWarmStartMatchDistance);
  EXPECT_EQ(crossStep.warmStartMaxAge, -1);

  std::array<Eigen::VectorXd, kCubeCount> positions;
  std::array<Eigen::VectorXd, kCubeCount> velocities;
  for (const auto& cubeSpec : makeCubeSpecs()) {
    const auto cube = exactWorld->getSkeleton(cubeSpec.name);
    ASSERT_NE(cube, nullptr);
    EXPECT_FALSE(cube->isMobile());
    positions[cubeSpec.index] = cube->getPositions();
    velocities[cubeSpec.index] = cube->getVelocities();
  }

  adapter::releaseCubes(exactWorld);
  EXPECT_TRUE(adapter::cubesReleased(exactWorld));
  EXPECT_TRUE(adapter::inspectAdapterContract(exactWorld).cubesAreReleased);
  for (const auto& cubeSpec : makeCubeSpecs()) {
    const auto cube = exactWorld->getSkeleton(cubeSpec.name);
    ASSERT_NE(cube, nullptr);
    EXPECT_TRUE(cube->isMobile());
    EXPECT_TRUE(cube->getPositions().isApprox(positions[cubeSpec.index], 0.0));
    EXPECT_TRUE(
        cube->getVelocities().isApprox(velocities[cubeSpec.index], 0.0));
  }

  // The completed-step action is intentionally idempotent: it never respawns,
  // teleports, or injects velocity into an already released cube.
  adapter::releaseCubes(exactWorld);
  EXPECT_EQ(exactWorld->getNumSkeletons(), 1u + kStoneCount + kCubeCount);

  const std::string json = adapter::adapterContractJson(
      exact, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  EXPECT_NE(json.find(adapter::kContractSchema), std::string::npos);
  EXPECT_NE(json.find(kAuthorCommit), std::string::npos);
  EXPECT_NE(
      json.find("\"coordinate_units\":\"author_raw_numeric_values\""),
      std::string::npos);
  EXPECT_NE(
      json.find("\"evidence_runner_action_completed_step\":1600"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"interactive_action_semantics\":\"immediate_on_invocation\""),
      std::string::npos);
  EXPECT_NE(json.find("\"step_size_scale\":35"), std::string::npos);
  EXPECT_NE(
      json.find("\"inner_local_solver\":\"exact_metric_projection\""),
      std::string::npos);
  const std::string boxedJson = adapter::adapterContractJson(
      boxed, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  EXPECT_NE(boxedJson.find("\"exact_options\":null"), std::string::npos);
  EXPECT_NE(boxedJson.find("\"cross_step_options\":null"), std::string::npos);
  EXPECT_EQ(boxedJson.find("\"max_outer_iterations\""), std::string::npos);
  EXPECT_NE(
      json.find("\"interactive_demo_auto_releases_at_source_step\":false"),
      std::string::npos);
  EXPECT_NE(json.find("\"standing_outcome_oracle\":null"), std::string::npos);
  EXPECT_NE(
      json.find(
          "\"current_dart_adapter_standing_outcome_oracle_declared\":false"),
      std::string::npos);
  EXPECT_NE(json.find("\"fig07_parity\":false"), std::string::npos);
  EXPECT_NE(json.find("\"paper_parity\":false"), std::string::npos);
}

TEST(FbfAuthorMasonryArchSpec, DartAdapterSourceContinuationLaneIsSeparate)
{
  namespace adapter = fbf_author_masonry_arch_adapter;
  using namespace fbf_author_masonry_arch;

  EXPECT_STREQ(
      adapter::kSourceContinuationDemoSceneId,
      "fbf_author_masonry_arch_25_crown_impact_source_continuation_current_"
      "source");
  EXPECT_STRNE(adapter::kSourceContinuationDemoSceneId, adapter::kDemoSceneId);
  EXPECT_STREQ(
      adapter::kSourceContinuationContractSchema,
      "dart.fbf_author_masonry_arch_crown_impact_source_continuation_dart_"
      "adapter/v1");

  const auto strictWorld = adapter::createWorld(adapter::SolverLane::ExactFbf);
  const auto exactWorld
      = adapter::createSourceContinuationWorld(adapter::SolverLane::ExactFbf);
  const auto boxedWorld
      = adapter::createSourceContinuationWorld(adapter::SolverLane::BoxedLcp);
  const auto strict = adapter::inspectAdapterContract(strictWorld);
  const auto exact = adapter::inspectAdapterContract(exactWorld);
  const auto boxed = adapter::inspectAdapterContract(boxedWorld);

  EXPECT_FALSE(strict.sourceContinuationScene);
  EXPECT_TRUE(exact.sourceContinuationScene);
  EXPECT_TRUE(boxed.sourceContinuationScene);
  EXPECT_FALSE(strict.exactSourceContinuationOptions.enabled);
  EXPECT_FALSE(strict.exactSourceContinuationActive);
  EXPECT_TRUE(exact.exactSourceContinuationOptions.enabled);
  EXPECT_FALSE(exact.exactSourceContinuationActive);
  EXPECT_FALSE(boxed.exactSourceContinuationOptions.enabled);
  EXPECT_FALSE(boxed.exactSourceContinuationActive);

  auto* strictSolver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          strictWorld->getConstraintSolver());
  auto* exactSolver
      = dynamic_cast<dart::constraint::ExactCoulombFbfConstraintSolver*>(
          exactWorld->getConstraintSolver());
  ASSERT_NE(strictSolver, nullptr);
  ASSERT_NE(exactSolver, nullptr);
  EXPECT_EQ(
      exactSolver->getExactCoulombPostCorrectionProjectionEnabled(),
      strictSolver->getExactCoulombPostCorrectionProjectionEnabled());
  EXPECT_EQ(
      exactSolver->getExactCoulombSourceInnerInitializationEnabled(),
      strictSolver->getExactCoulombSourceInnerInitializationEnabled());

  for (const auto* continuation : {&exact, &boxed}) {
    EXPECT_DOUBLE_EQ(continuation->timeStep, strict.timeStep);
    EXPECT_TRUE(continuation->gravity.isApprox(strict.gravity, 0.0));
    EXPECT_EQ(continuation->simulationThreads, strict.simulationThreads);
    EXPECT_EQ(continuation->deactivationEnabled, strict.deactivationEnabled);
    EXPECT_EQ(continuation->maxContacts, strict.maxContacts);
    EXPECT_EQ(continuation->maxContactsPerPair, strict.maxContactsPerPair);
    EXPECT_EQ(
        continuation->nativeFourPointPlanar, strict.nativeFourPointPlanar);
    EXPECT_EQ(continuation->splitImpulseEnabled, strict.splitImpulseEnabled);
    EXPECT_EQ(continuation->stoneCount, strict.stoneCount);
    EXPECT_EQ(continuation->mobileStoneCount, strict.mobileStoneCount);
    EXPECT_EQ(continuation->cubeCount, strict.cubeCount);
    EXPECT_EQ(continuation->cubesAreReleased, strict.cubesAreReleased);
  }

  ASSERT_TRUE(exact.exactOptions.has_value());
  ASSERT_TRUE(exact.exactCrossStepOptions.has_value());
  const auto& options = *exact.exactOptions;
  EXPECT_FALSE(options.fallbackToBoxedLcp);
  EXPECT_FALSE(options.enableStepSizePersistence);
  EXPECT_FALSE(options.seedNormalImpulseFromDiagonal);
  EXPECT_FALSE(options.useMatrixFreeDelassusSeed);
  EXPECT_FALSE(options.enableProjectedGradientRetry);
  EXPECT_FALSE(options.enableDenseResidualPolish);
  EXPECT_EQ(
      options.maxOuterIterations,
      adapter::kSourceContinuationMaxOuterIterations);
  EXPECT_FALSE(options.acceptOuterMaxIterations);
  EXPECT_DOUBLE_EQ(
      options.warmStartMatchDistance,
      adapter::kSourceContinuationWarmStartMatchDistance);
  EXPECT_DOUBLE_EQ(options.stepSizeScale, adapter::kDartStepSizeScale);
  EXPECT_DOUBLE_EQ(options.outerRelaxation, adapter::kDartOuterRelaxation);
  EXPECT_EQ(options.innerMaxSweeps, adapter::kDartInnerMaxSweeps);
  EXPECT_EQ(options.innerLocalIterations, adapter::kDartInnerLocalIterations);
  EXPECT_EQ(
      options.maxResidualHistorySamples,
      adapter::kSourceContinuationResidualHistorySamples);
  EXPECT_EQ(
      options.maxResidualHistoryRecords,
      adapter::kSourceContinuationResidualHistoryRecords);

  const auto& continuation = exact.exactSourceContinuationOptions;
  EXPECT_EQ(
      continuation.residualCheckInterval,
      adapter::kSourceContinuationResidualCheckInterval);
  EXPECT_EQ(
      continuation.plateauPatience,
      adapter::kSourceContinuationPlateauPatience);
  EXPECT_DOUBLE_EQ(
      continuation.plateauRelativeTolerance,
      adapter::kSourceContinuationPlateauRelativeTolerance);
  EXPECT_EQ(
      continuation.stepSizeBacktrackLimit,
      adapter::kSourceContinuationStepSizeBacktrackLimit);
  EXPECT_DOUBLE_EQ(
      continuation.couplingVariationSkipThreshold,
      adapter::kSourceContinuationCouplingVariationSkipThreshold);

  const auto& crossStep = *exact.exactCrossStepOptions;
  EXPECT_EQ(
      crossStep.warmStartMatchMode,
      dart::constraint::ExactCoulombFbfWarmStartMatchMode::
          OrderedBodyBLocalFeature);
  EXPECT_TRUE(crossStep.useStrictWarmStartMatchDistance);
  EXPECT_DOUBLE_EQ(
      crossStep.warmStartNormalCosine,
      adapter::kSourceContinuationWarmStartNormalCosine);
  EXPECT_EQ(
      crossStep.warmStartMaxAge, adapter::kSourceContinuationWarmStartMaxAge);
  EXPECT_DOUBLE_EQ(
      crossStep.persistentStepSizeSafeBoundScale,
      adapter::kSourceContinuationPersistentStepSizeSafeBoundScale);
  EXPECT_DOUBLE_EQ(
      crossStep.minimumStepSize, adapter::kSourceContinuationMinimumStepSize);
  EXPECT_DOUBLE_EQ(
      crossStep.maximumStepSize, adapter::kSourceContinuationMaximumStepSize);
  EXPECT_DOUBLE_EQ(
      crossStep.warmStartResidualThreshold,
      adapter::kSourceContinuationWarmStartResidualThreshold);
  EXPECT_DOUBLE_EQ(
      crossStep.warmStartStepSizeCap,
      adapter::kSourceContinuationWarmStartStepSizeCap);
  EXPECT_TRUE(crossStep.persistUncappedStepSizeOnWarmStartCap);
  EXPECT_TRUE(crossStep.requireResidualImprovementForUnconvergedCacheSave);

  EXPECT_FALSE(boxed.exactOptions.has_value());
  EXPECT_FALSE(boxed.exactCrossStepOptions.has_value());
  EXPECT_EQ(boxed.solverLane, adapter::SolverLane::BoxedLcp);

  const std::string strictJson = adapter::adapterContractJson(
      strict, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  const std::string exactJson = adapter::adapterContractJson(
      exact, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  const std::string boxedJson = adapter::adapterContractJson(
      boxed, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  EXPECT_EQ(strictJson.find("\"source_continuation\""), std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"source_continuation\":{\"policy\":\"source_continuation\""),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"options_available\":true,\"requested\":true"),
      std::string::npos);
  EXPECT_NE(
      boxedJson.find("\"options_available\":false,\"requested\":false"),
      std::string::npos);
  EXPECT_NE(boxedJson.find("\"exact_options\":null"), std::string::npos);
}

TEST(FbfAuthorMasonryArchSpec, DartAdapterPinsArch101StandingContract)
{
  namespace adapter = fbf_author_masonry_arch_adapter;
  using namespace fbf_author_masonry_arch;

  EXPECT_STREQ(
      adapter::kDemoSceneId101,
      "fbf_author_masonry_arch_101_standing_current_source");
  EXPECT_STREQ(
      adapter::kContractSchema101,
      "dart.fbf_author_masonry_arch_standing_dart_adapter/v1");
  EXPECT_EQ(adapter::kEvidenceFrameCount101, 400u);
  EXPECT_EQ(adapter::kEvidenceTotalSubsteps101, 1600u);
  EXPECT_FALSE(
      adapter::kAdapterScenario101.evidenceRunnerReleaseActionScheduled);

  const auto exactWorld = adapter::createWorld(
      adapter::SolverLane::ExactFbf, SourceScenario::Arch101);
  const auto boxedWorld = adapter::createWorld(
      adapter::SolverLane::BoxedLcp, SourceScenario::Arch101);
  EXPECT_EQ(exactWorld->getName(), adapter::kDemoSceneId101);
  EXPECT_EQ(boxedWorld->getName(), adapter::kDemoSceneId101);

  const auto exact
      = adapter::inspectAdapterContract(exactWorld, SourceScenario::Arch101);
  const auto boxed
      = adapter::inspectAdapterContract(boxedWorld, SourceScenario::Arch101);
  for (const auto* contract : {&exact, &boxed}) {
    EXPECT_EQ(contract->sourceScenario, SourceScenario::Arch101);
    EXPECT_DOUBLE_EQ(contract->timeStep, kRuntimeTimeStep);
    EXPECT_TRUE(contract->gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
    EXPECT_FALSE(contract->deactivationEnabled);
    EXPECT_EQ(contract->maxContacts, kSourceMaxContacts);
    EXPECT_EQ(contract->maxContactsPerPair, adapter::kDartMaxContactsPerPair);
    EXPECT_TRUE(contract->nativeFourPointPlanar);
    EXPECT_TRUE(contract->splitImpulseEnabled);
    EXPECT_FALSE(contract->cubesAreReleased);
    EXPECT_EQ(contract->stoneCount, kStoneCount101);
    EXPECT_EQ(contract->mobileStoneCount, kStoneCount101 - kFixedSpringerCount);
    EXPECT_EQ(contract->cubeCount, kCubeCount);
  }
  EXPECT_EQ(exact.solverLane, adapter::SolverLane::ExactFbf);
  EXPECT_TRUE(exact.exactOptions.has_value());
  EXPECT_TRUE(exact.exactCrossStepOptions.has_value());
  EXPECT_EQ(boxed.solverLane, adapter::SolverLane::BoxedLcp);
  EXPECT_FALSE(boxed.exactOptions.has_value());
  EXPECT_FALSE(boxed.exactCrossStepOptions.has_value());

  for (const auto& cubeSpec : makeCubeSpecs(SourceScenario::Arch101)) {
    const auto cube = exactWorld->getSkeleton(cubeSpec.name);
    ASSERT_NE(cube, nullptr);
    EXPECT_FALSE(cube->isMobile());
    EXPECT_DOUBLE_EQ(
        cube->getBodyNode(0u)->getWorldTransform().translation().z(),
        kCubeInitialZ101);
  }

  const std::string exactJson = adapter::adapterContractJson(
      exact, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  EXPECT_NE(exactJson.find(adapter::kContractSchema101), std::string::npos);
  EXPECT_NE(exactJson.find(kAuthorMeshTree101), std::string::npos);
  EXPECT_NE(exactJson.find(kAuthorMeshTreeSha256101), std::string::npos);
  EXPECT_NE(exactJson.find(kAuthorMeshDirectory101), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"selected_cli_arguments\":\"--stones 101\""),
      std::string::npos);
  EXPECT_NE(exactJson.find("\"selected_stones\":101"), std::string::npos);
  EXPECT_NE(exactJson.find("\"stones\":101"), std::string::npos);
  EXPECT_NE(exactJson.find("\"evidence_frames\":400"), std::string::npos);
  EXPECT_NE(exactJson.find("\"evidence_substeps\":1600"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"source_release_within_evidence_horizon\":false"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"evidence_runner_release_action_scheduled\":false"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"evidence_runner_action_completed_step\":null"),
      std::string::npos);
  EXPECT_NE(exactJson.find("\"release_action_key\":null"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"historical_paper_invocation_known\":false"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"source_release_action_ported_to_dart\":false"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"scene_state_schema\":\"dart.fbf_author_masonry_arch_101_"
          "standing_scene_state/v1\""),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"required_completed_substeps\":1600"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"max_mobile_body_origin_displacement\":3"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"max_mobile_rotation_delta_rad\":0.261799"),
      std::string::npos);
  EXPECT_NE(exactJson.find("\"max_crown_height_loss\":3"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"requires_exact_inventory\":true"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"requires_all_bodies_finite\":true"), std::string::npos);
  EXPECT_NE(
      exactJson.find("\"requires_cubes_kinematic\":true"), std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"max_kinematic_cube_pose_error\":9.9999999999999998e-13"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"requires_cubes_at_pinned_poses\":true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"positive_standing_qualification_requires_standing_in_both_"
          "lanes\":true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"comparison_capture_validity_is_runner_level\":true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"complete_trace_valid_is_not_positive_standing_qualification\":"
          "true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"current_dart_adapter_outcome_only\":true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"source_outcome_equivalence\":false"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find(
          "\"current_dart_adapter_standing_outcome_oracle_declared\":true"),
      std::string::npos);
  EXPECT_NE(
      exactJson.find("\"physical_outcome_equivalent\":false"),
      std::string::npos);
  EXPECT_NE(exactJson.find("\"fig08_parity\":false"), std::string::npos);
  EXPECT_NE(exactJson.find("\"video08_parity\":false"), std::string::npos);
  EXPECT_NE(exactJson.find("\"paper_parity\":false"), std::string::npos);

  const std::string boxedJson = adapter::adapterContractJson(
      boxed, kSpecSourceSha256, std::string(64u, 'a'), std::string(64u, 'b'));
  EXPECT_NE(boxedJson.find("\"exact_options\":null"), std::string::npos);
  EXPECT_NE(boxedJson.find("\"cross_step_options\":null"), std::string::npos);
}

TEST(FbfAuthorMasonryArchSpec, Arch101StandingStateAndOracleFailClosed)
{
  namespace adapter = fbf_author_masonry_arch_adapter;
  using fbf_author_masonry_arch::SourceScenario;

  const auto world = adapter::createWorld(
      adapter::SolverLane::BoxedLcp, SourceScenario::Arch101);
  const auto initial = adapter::inspectStandingSceneState(world);
  EXPECT_EQ(initial.solverLane, adapter::SolverLane::BoxedLcp);
  EXPECT_DOUBLE_EQ(initial.worldTimeSeconds, 0.0);
  EXPECT_EQ(initial.worldSkeletonCount, 105u);
  EXPECT_EQ(initial.observedStoneCount, 101u);
  EXPECT_EQ(initial.observedMobileStoneCount, 99u);
  EXPECT_EQ(initial.mobilityMatchingStoneCount, 101u);
  EXPECT_EQ(initial.finiteStoneCount, 101u);
  EXPECT_EQ(initial.observedCubeCount, 3u);
  EXPECT_EQ(initial.finiteCubeCount, 3u);
  EXPECT_EQ(initial.kinematicCubeCount, 3u);
  EXPECT_TRUE(initial.groundValid);
  EXPECT_TRUE(initial.crownObserved);
  EXPECT_NEAR(initial.crownBodyOriginZ, initial.crownBodyOriginInitialZ, 1e-12);
  EXPECT_NEAR(initial.crownBodyOriginDisplacement, 0.0, 1e-12);
  EXPECT_NEAR(initial.crownRotationDeltaRadians, 0.0, 1e-12);
  EXPECT_NEAR(initial.maxMobileBodyOriginDisplacement, 0.0, 1e-12);
  EXPECT_NEAR(initial.maxMobileRotationDeltaRadians, 0.0, 1e-12);
  EXPECT_NEAR(initial.maxKinematicCubeBodyOriginDisplacement, 0.0, 1e-12);
  EXPECT_NEAR(initial.maxKinematicCubeRotationDeltaRadians, 0.0, 1e-12);

  const auto initialOutcome = adapter::evaluateStandingOutcome(initial);
  EXPECT_FALSE(initialOutcome.horizonComplete);
  EXPECT_TRUE(initialOutcome.inventoryValid);
  EXPECT_TRUE(initialOutcome.allBodiesFinite);
  EXPECT_TRUE(initialOutcome.cubesRemainKinematic);
  EXPECT_TRUE(initialOutcome.cubesRemainPinned);
  EXPECT_TRUE(initialOutcome.mobileDisplacementBounded);
  EXPECT_TRUE(initialOutcome.mobileRotationBounded);
  EXPECT_TRUE(initialOutcome.crownHeightPreserved);
  EXPECT_FALSE(initialOutcome.completeTraceValid);
  EXPECT_FALSE(initialOutcome.standingOutcomeValid);
  EXPECT_FALSE(initialOutcome.laneEvidenceQualifies);

  const std::vector<std::string> expectedKeys{
      "solver_lane_exact_fbf",
      "solver_lane_boxed_lcp",
      "world_time_seconds",
      "world_skeleton_count",
      "observed_stone_count",
      "observed_mobile_stone_count",
      "mobility_matching_stone_count",
      "finite_stone_count",
      "observed_cube_count",
      "finite_cube_count",
      "kinematic_cube_count",
      "ground_valid",
      "crown_observed",
      "crown_body_origin_initial_z",
      "crown_body_origin_z",
      "crown_body_origin_displacement",
      "crown_rotation_delta_rad",
      "max_mobile_body_origin_displacement",
      "max_mobile_rotation_delta_rad",
      "max_kinematic_cube_body_origin_displacement",
      "max_kinematic_cube_rotation_delta_rad",
      "standing_horizon_complete",
      "standing_inventory_valid",
      "standing_all_bodies_finite",
      "standing_cubes_remain_kinematic",
      "standing_cubes_remain_pinned",
      "standing_mobile_displacement_bounded",
      "standing_mobile_rotation_bounded",
      "standing_crown_height_preserved",
      "standing_complete_trace_valid",
      "standing_outcome_valid",
      "standing_lane_evidence_qualifies",
  };
  const auto fields = adapter::standingSceneStateFields(initial);
  ASSERT_EQ(fields.size(), expectedKeys.size());
  for (std::size_t index = 0u; index < fields.size(); ++index) {
    EXPECT_EQ(fields[index].first, expectedKeys[index]);
    EXPECT_TRUE(std::isfinite(fields[index].second));
  }
  EXPECT_DOUBLE_EQ(fields[0].second, 0.0);
  EXPECT_DOUBLE_EQ(fields[1].second, 1.0);
  EXPECT_DOUBLE_EQ(fields[21].second, 0.0);
  EXPECT_DOUBLE_EQ(fields[30].second, 0.0);
  EXPECT_DOUBLE_EQ(fields[31].second, 0.0);

  auto complete = initial;
  complete.worldTimeSeconds = adapter::kStandingRequiredWorldTimeSeconds;
  complete.maxMobileBodyOriginDisplacement
      = adapter::kStandingMaxMobileBodyOriginDisplacement;
  complete.maxMobileRotationDeltaRadians
      = adapter::kStandingMaxMobileRotationDeltaRadians;
  complete.crownBodyOriginZ
      = complete.crownBodyOriginInitialZ - adapter::kStandingMaxCrownHeightLoss;
  const auto completeOutcome = adapter::evaluateStandingOutcome(complete);
  EXPECT_TRUE(completeOutcome.completeTraceValid);
  EXPECT_TRUE(completeOutcome.standingOutcomeValid);
  EXPECT_TRUE(completeOutcome.laneEvidenceQualifies);
  EXPECT_DOUBLE_EQ(
      adapter::standingSceneStateFields(complete).back().second, 1.0);

  auto incomplete = complete;
  incomplete.worldTimeSeconds
      = adapter::kStandingRequiredWorldTimeSeconds
        - 2.0 * adapter::kStandingHorizonTimeToleranceSeconds;
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(incomplete).standingOutcomeValid);

  auto displaced = complete;
  displaced.maxMobileBodyOriginDisplacement = std::nextafter(
      adapter::kStandingMaxMobileBodyOriginDisplacement,
      std::numeric_limits<double>::infinity());
  const auto displacedOutcome = adapter::evaluateStandingOutcome(displaced);
  EXPECT_TRUE(displacedOutcome.completeTraceValid);
  EXPECT_FALSE(displacedOutcome.standingOutcomeValid);
  EXPECT_FALSE(displacedOutcome.laneEvidenceQualifies);

  auto rotated = complete;
  rotated.maxMobileRotationDeltaRadians = std::nextafter(
      adapter::kStandingMaxMobileRotationDeltaRadians,
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(adapter::evaluateStandingOutcome(rotated).standingOutcomeValid);

  auto crownDropped = complete;
  crownDropped.crownBodyOriginZ = std::nextafter(
      complete.crownBodyOriginInitialZ - adapter::kStandingMaxCrownHeightLoss,
      -std::numeric_limits<double>::infinity());
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(crownDropped).standingOutcomeValid);

  auto nonfinite = complete;
  nonfinite.maxMobileRotationDeltaRadians
      = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::evaluateStandingOutcome(nonfinite).allBodiesFinite);
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(nonfinite).standingOutcomeValid);

  auto movedCube = complete;
  movedCube.maxKinematicCubeBodyOriginDisplacement = std::nextafter(
      adapter::kStandingMaxKinematicCubePoseError,
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(adapter::evaluateStandingOutcome(movedCube).cubesRemainPinned);
  EXPECT_FALSE(adapter::evaluateStandingOutcome(movedCube).completeTraceValid);
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(movedCube).standingOutcomeValid);

  auto missingStone = complete;
  --missingStone.observedStoneCount;
  EXPECT_FALSE(adapter::evaluateStandingOutcome(missingStone).inventoryValid);
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(missingStone).standingOutcomeValid);

  auto releasedCube = complete;
  --releasedCube.kinematicCubeCount;
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(releasedCube).cubesRemainKinematic);
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(releasedCube).cubesRemainPinned);
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(releasedCube).standingOutcomeValid);

  auto exactComplete = complete;
  exactComplete.solverLane = adapter::SolverLane::ExactFbf;
  EXPECT_TRUE(
      adapter::evaluateStandingOutcome(exactComplete).laneEvidenceQualifies);
  exactComplete.maxMobileRotationDeltaRadians = std::nextafter(
      adapter::kStandingMaxMobileRotationDeltaRadians,
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(
      adapter::evaluateStandingOutcome(exactComplete).laneEvidenceQualifies);

  const auto wrongWorld = adapter::createWorld(adapter::SolverLane::BoxedLcp);
  EXPECT_THROW(
      adapter::inspectStandingSceneState(wrongWorld), std::runtime_error);
}

TEST(FbfAuthorMasonryArchSpec, DartAdapterScopesContactErp)
{
  namespace adapter = fbf_author_masonry_arch_adapter;
  const double previous
      = dart::constraint::ContactConstraint::getErrorReductionParameter();
  {
    adapter::ScopedContactErrorReductionParameter scoped;
    EXPECT_DOUBLE_EQ(
        dart::constraint::ContactConstraint::getErrorReductionParameter(),
        adapter::kDartContactErrorReductionParameter);
  }
  EXPECT_DOUBLE_EQ(
      dart::constraint::ContactConstraint::getErrorReductionParameter(),
      previous);
}

} // namespace
