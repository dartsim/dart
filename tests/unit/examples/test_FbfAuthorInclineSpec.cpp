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

#include "../../../examples/demos/scenes/FbfAuthorInclineSpec.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

using namespace fbf_author_incline;

//==============================================================================
TEST(FbfAuthorInclineSpec, SourceGridNamesAndLayoutArePinned)
{
  EXPECT_STREQ(kSceneId, "fbf_author_incline_sweep_current_source");
  EXPECT_STREQ(kAuthorCommit, "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0");
  EXPECT_STREQ(kAuthorTree, "ffcdafb61adeda2239c8366d054b548b50d26685");
  EXPECT_STREQ(kAuthorRunBlob, "63cfc28dca1f6c65ce4a27dbfa239cba154580c6");
  EXPECT_STREQ(
      kAuthorRunSha256,
      "881d486f25d85f9ae197bf4164e110b6bc39775c498433f27ec4407ca09ebf82");
  EXPECT_DOUBLE_EQ(kSourceDefaultFriction, 0.5);
  EXPECT_STREQ(kMuGridProvenanceKind, "operator_selected_figure_1_grid");
  EXPECT_STREQ(kMuGridSelectionInterface, "supported_--mu_cli");

  constexpr std::array<double, kCellCount> kExpectedFriction{
      {0.3, 0.4, 0.45, 0.5, 0.55, 0.6, 0.8}};
  constexpr std::array<const char*, kCellCount> kExpectedPrefixes{{
      "mu_0_3",
      "mu_0_4",
      "mu_0_45",
      "mu_0_5",
      "mu_0_55",
      "mu_0_6",
      "mu_0_8",
  }};
  std::set<std::string> names;
  for (std::size_t index = 0u; index < kCells.size(); ++index) {
    const auto& cell = kCells[index];
    EXPECT_EQ(cell.index, index);
    EXPECT_DOUBLE_EQ(cell.friction, kExpectedFriction[index]);
    EXPECT_STREQ(cell.statePrefix, kExpectedPrefixes[index]);
    EXPECT_DOUBLE_EQ(
        cell.laneTranslationY,
        (static_cast<double>(index) - 3.0) * kLaneSpacing);
    EXPECT_EQ(
        cell.planeSkeletonName,
        std::string("author_incline_plane_") + cell.statePrefix);
    EXPECT_EQ(
        cell.planeBodyName,
        std::string("author_incline_plane_") + cell.statePrefix + "_body");
    EXPECT_EQ(
        cell.cubeSkeletonName,
        std::string("author_incline_cube_") + cell.statePrefix);
    EXPECT_EQ(
        cell.cubeBodyName,
        std::string("author_incline_cube_") + cell.statePrefix + "_body");
    EXPECT_EQ(findCellByCubeSkeletonName(cell.cubeSkeletonName), &cell);
    names.insert(cell.planeSkeletonName);
    names.insert(cell.planeBodyName);
    names.insert(cell.cubeSkeletonName);
    names.insert(cell.cubeBodyName);
  }
  EXPECT_EQ(names.size(), 4u * kCellCount);
  EXPECT_EQ(findCellByCubeSkeletonName("author_incline_cube_mu_0_35"), nullptr);
}

//==============================================================================
TEST(FbfAuthorInclineSpec, PublishedGeometryAndClockArePinned)
{
  EXPECT_DOUBLE_EQ(kInclineTangent, 0.5);
  EXPECT_NEAR(std::tan(kInclineAngleRadians), kInclineTangent, 1e-15);
  EXPECT_TRUE(planeSize().isApprox(Eigen::Vector3d(10.0, 3.0, 0.1), 0.0));
  EXPECT_TRUE(cubeSize().isApprox(Eigen::Vector3d::Ones(), 0.0));
  EXPECT_DOUBLE_EQ(kCubeDensity, 1000.0);
  EXPECT_DOUBLE_EQ(cubeMass(), 1000.0);
  EXPECT_TRUE(
      cubeMoment().isApprox(Eigen::Matrix3d::Identity() * (1000.0 / 6.0), 0.0));
  EXPECT_DOUBLE_EQ(kInitialGeometricSeparation, 0.001);
  EXPECT_DOUBLE_EQ(kInitialCenterNormalDistance, 0.501);
  EXPECT_DOUBLE_EQ(kSourceCubeShapeGap, 0.01);
  EXPECT_DOUBLE_EQ(kTimeStep, 1.0 / 60.0);
  EXPECT_DOUBLE_EQ(kDuration, 2.0);
  EXPECT_EQ(kTotalSteps, 120u);
  EXPECT_DOUBLE_EQ(kTotalSteps * kTimeStep, kDuration);
  EXPECT_FALSE(kDartFallbackToBoxedLcpEnabled);

  EXPECT_TRUE(sourceRotation().isApprox(
      Eigen::AngleAxisd(kInclineAngleRadians, Eigen::Vector3d::UnitY())
          .toRotationMatrix(),
      0.0));
  EXPECT_TRUE(slopeTangent().isApprox(
      sourceRotation() * Eigen::Vector3d::UnitX(), 1e-15));
  EXPECT_TRUE(slopeNormal().isApprox(
      sourceRotation() * Eigen::Vector3d::UnitZ(), 1e-15));
  for (const auto& cell : kCells) {
    const auto plane = planePose(cell);
    const auto cube = cubePose(cell);
    EXPECT_TRUE(plane.linear().isApprox(sourceRotation(), 0.0));
    EXPECT_TRUE(cube.linear().isApprox(sourceRotation(), 0.0));
    EXPECT_DOUBLE_EQ(plane.translation().y(), cell.laneTranslationY);
    EXPECT_DOUBLE_EQ(cube.translation().y(), cell.laneTranslationY);
    EXPECT_NEAR(
        slopeNormal().dot(cube.translation()),
        kInitialCenterNormalDistance,
        1e-15);
  }
}

//==============================================================================
TEST(FbfAuthorInclineSpec, SceneStateSchemaIsDeterministic)
{
  const auto names = sceneStateFieldNames();
  EXPECT_EQ(names.size(), 1u + 19u * kCellCount);
  EXPECT_EQ(names.front(), "world_time_seconds");
  EXPECT_EQ(names[1], "mu_0_3_mu");
  EXPECT_EQ(names[5], "mu_0_3_orientation_w");
  EXPECT_EQ(names[12], "mu_0_3_angular_velocity_x_rad_s");
  EXPECT_EQ(names[19], "mu_0_3_contact_count");
  EXPECT_EQ(names[20], "mu_0_4_mu");
  EXPECT_EQ(names.back(), "mu_0_8_contact_count");
  EXPECT_EQ(
      std::set<std::string>(names.begin(), names.end()).size(), names.size());
}

//==============================================================================
TEST(FbfAuthorInclineSpec, ContractValidationFailsClosed)
{
  EXPECT_THROW(
      inspectPhysicsContract(nullptr, std::string(64u, '0')),
      std::runtime_error);
  PhysicsContract incomplete;
  EXPECT_THROW(physicsContractJson(incomplete), std::runtime_error);
}

//==============================================================================
TEST(FbfAuthorInclineSpec, ProductionWorldWiresBothSolverLanesAndState)
{
  const std::array<std::pair<SolverLane, std::string>, 2> lanes{{
      {SolverLane::ExactFbf, "exact_fbf"},
      {SolverLane::BoxedLcp, "boxed_lcp"},
  }};

  for (const auto& lane : lanes) {
    const auto world = createWorld(lane.first);
    ASSERT_NE(world, nullptr);

    const auto contract = inspectPhysicsContract(world, std::string(64u, 'a'));
    EXPECT_EQ(contract.solverLane, lane.second);
    EXPECT_EQ(contract.cells.size(), kCellCount);
    EXPECT_EQ(contract.maxContacts, kMaxContacts);
    EXPECT_EQ(contract.maxContactsPerPair, kMaxContactsPerPair);
    EXPECT_FALSE(contract.splitImpulseEnabled);
    EXPECT_EQ(
        contract.exactOptionsAvailable, lane.first == SolverLane::ExactFbf);
    if (lane.first == SolverLane::ExactFbf) {
      EXPECT_FALSE(contract.fallbackToBoxedLcpEnabled);
    }

    const std::string contractJson = physicsContractJson(contract);
    EXPECT_NE(
        contractJson.find("\"lane\":\"" + lane.second + "\""),
        std::string::npos);

    const auto initialFields = sceneStateFields(world);
    ASSERT_EQ(initialFields.size(), sceneStateFieldNames().size());
    const std::map<std::string, double> initialState(
        initialFields.begin(), initialFields.end());
    ASSERT_EQ(initialState.size(), initialFields.size());
    EXPECT_DOUBLE_EQ(initialState.at("world_time_seconds"), 0.0);
    for (const auto& cell : kCells) {
      const std::string prefix(cell.statePrefix);
      EXPECT_DOUBLE_EQ(initialState.at(prefix + "_mu"), cell.friction);
      EXPECT_NEAR(
          initialState.at(prefix + "_normal_distance_m"),
          kInitialCenterNormalDistance,
          1e-12);
      EXPECT_DOUBLE_EQ(initialState.at(prefix + "_contact_count"), 0.0);
    }

    EXPECT_NO_THROW(world->step());
    EXPECT_NO_THROW(world->step());
    EXPECT_NEAR(world->getTime(), 2.0 * kTimeStep, 1e-15);
    EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), kMaxContacts);

    const auto steppedFields = sceneStateFields(world);
    ASSERT_EQ(steppedFields.size(), initialFields.size());
    const std::map<std::string, double> steppedState(
        steppedFields.begin(), steppedFields.end());
    ASSERT_EQ(steppedState.size(), steppedFields.size());
    for (const auto& field : steppedState)
      EXPECT_TRUE(std::isfinite(field.second)) << field.first;
    for (const auto& cell : kCells) {
      EXPECT_DOUBLE_EQ(
          steppedState.at(std::string(cell.statePrefix) + "_contact_count"),
          static_cast<double>(kMaxContactsPerPair));
    }
  }
}

} // namespace
