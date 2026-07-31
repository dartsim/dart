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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "examples/demos/scenes/AdaptiveSoftContactModel.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>

#include <cstddef>

namespace {

namespace model = dart_demos::adaptive_soft_contact_model;

constexpr std::size_t kVerificationSteps = 2000;
constexpr std::size_t kCheckpointInterval = 250;
constexpr std::size_t kExpectedPointMasses = 86;

struct Snapshot
{
  std::size_t step = 0;
  std::size_t adaptiveActive = 0;
  std::size_t adaptiveContacts = 0;
  std::size_t allActive = 0;
  std::size_t allActiveContacts = 0;
  double adaptivePositionChecksum = 0.0;
  double allActivePositionChecksum = 0.0;
  double surfacePoseDelta = 0.0;
};

struct Outcome
{
  bool finite = true;
  std::size_t nonFiniteStep = 0;
  std::size_t total = 0;
  std::size_t reducedSteps = 0;
  std::size_t adaptiveContactSteps = 0;
  std::size_t allActiveContactSteps = 0;
  std::size_t finalActive = 0;
  double maximumSurfacePoseDelta = 0.0;
  double finalAdaptivePositionChecksum = 0.0;
  double finalAllActivePositionChecksum = 0.0;
  std::vector<Snapshot> checkpoints;
};

Outcome runComparison()
{
  auto adaptive = model::createModel(true);
  auto allActive = model::createModel(false);
  adaptive.world->setNumSimulationThreads(1);
  allActive.world->setNumSimulationThreads(1);

  Outcome outcome;
  outcome.total = adaptive.softBody->getNumPointMasses();
  outcome.checkpoints.reserve(kVerificationSteps / kCheckpointInterval);

  for (std::size_t step = 1; step <= kVerificationSteps; ++step) {
    const bool adaptiveFinite = model::step(adaptive);
    const bool allActiveFinite = model::step(allActive);
    if (!adaptiveFinite || !allActiveFinite) {
      outcome.finite = false;
      outcome.nonFiniteStep = step;
      return outcome;
    }

    const std::size_t active = adaptive.softBody->getNumActivePointMasses();
    if (active < outcome.total)
      ++outcome.reducedSteps;
    if (adaptive.world->getLastCollisionResult().getNumContacts() > 0)
      ++outcome.adaptiveContactSteps;
    if (allActive.world->getLastCollisionResult().getNumContacts() > 0)
      ++outcome.allActiveContactSteps;

    const double poseDelta = model::surfacePoseDelta(adaptive, allActive);
    outcome.maximumSurfacePoseDelta
        = std::max(outcome.maximumSurfacePoseDelta, poseDelta);

    if (step % kCheckpointInterval == 0) {
      const model::Checksum adaptiveChecksum = model::computeChecksum(adaptive);
      const model::Checksum allActiveChecksum
          = model::computeChecksum(allActive);
      outcome.checkpoints.push_back(Snapshot{
          step,
          adaptiveChecksum.active,
          adaptiveChecksum.contacts,
          allActiveChecksum.active,
          allActiveChecksum.contacts,
          model::positionChecksum(adaptive),
          model::positionChecksum(allActive),
          poseDelta});
    }
  }

  outcome.finalActive = adaptive.softBody->getNumActivePointMasses();
  outcome.finalAdaptivePositionChecksum = model::positionChecksum(adaptive);
  outcome.finalAllActivePositionChecksum = model::positionChecksum(allActive);
  return outcome;
}

void expectExactlyEqual(const Outcome& lhs, const Outcome& rhs)
{
  ASSERT_EQ(lhs.checkpoints.size(), rhs.checkpoints.size());
  EXPECT_EQ(lhs.finite, rhs.finite);
  EXPECT_EQ(lhs.nonFiniteStep, rhs.nonFiniteStep);
  EXPECT_EQ(lhs.total, rhs.total);
  EXPECT_EQ(lhs.reducedSteps, rhs.reducedSteps);
  EXPECT_EQ(lhs.adaptiveContactSteps, rhs.adaptiveContactSteps);
  EXPECT_EQ(lhs.allActiveContactSteps, rhs.allActiveContactSteps);
  EXPECT_EQ(lhs.finalActive, rhs.finalActive);
  EXPECT_EQ(lhs.maximumSurfacePoseDelta, rhs.maximumSurfacePoseDelta);
  EXPECT_EQ(
      lhs.finalAdaptivePositionChecksum, rhs.finalAdaptivePositionChecksum);
  EXPECT_EQ(
      lhs.finalAllActivePositionChecksum, rhs.finalAllActivePositionChecksum);

  for (std::size_t i = 0; i < lhs.checkpoints.size(); ++i) {
    const auto& left = lhs.checkpoints[i];
    const auto& right = rhs.checkpoints[i];
    EXPECT_EQ(left.step, right.step) << "checkpoint " << i;
    EXPECT_EQ(left.adaptiveActive, right.adaptiveActive) << "checkpoint " << i;
    EXPECT_EQ(left.adaptiveContacts, right.adaptiveContacts)
        << "checkpoint " << i;
    EXPECT_EQ(left.allActive, right.allActive) << "checkpoint " << i;
    EXPECT_EQ(left.allActiveContacts, right.allActiveContacts)
        << "checkpoint " << i;
    EXPECT_EQ(left.adaptivePositionChecksum, right.adaptivePositionChecksum)
        << "checkpoint " << i;
    EXPECT_EQ(left.allActivePositionChecksum, right.allActivePositionChecksum)
        << "checkpoint " << i;
    EXPECT_EQ(left.surfacePoseDelta, right.surfacePoseDelta)
        << "checkpoint " << i;
  }
}

} // namespace

//==============================================================================
TEST(AdaptiveSoftContactModelTest, PreservesDeterministicComparisonContract)
{
  const Outcome first = runComparison();
  ASSERT_TRUE(first.finite)
      << "first comparison became non-finite at step " << first.nonFiniteStep;
  ASSERT_EQ(first.total, kExpectedPointMasses);
  ASSERT_EQ(first.checkpoints.size(), kVerificationSteps / kCheckpointInterval);
  EXPECT_GT(first.reducedSteps, kVerificationSteps / 2);
  EXPECT_GT(first.adaptiveContactSteps, 0u);
  EXPECT_GT(first.allActiveContactSteps, 0u);
  EXPECT_LE(first.finalActive * 4, first.total * 3)
      << "adaptive activation did not leave a meaningful inactive region";
  for (const auto& checkpoint : first.checkpoints)
    EXPECT_EQ(checkpoint.allActive, first.total);
  EXPECT_LE(first.maximumSurfacePoseDelta, model::kCompareTolerance);

  const Outcome second = runComparison();
  ASSERT_TRUE(second.finite)
      << "second comparison became non-finite at step " << second.nonFiniteStep;
  expectExactlyEqual(first, second);

  std::cout << std::setprecision(17)
            << "adaptive_soft_contact steps=" << kVerificationSteps
            << " total=" << first.total << " final_active=" << first.finalActive
            << " reduced_steps=" << first.reducedSteps
            << " adaptive_contact_steps=" << first.adaptiveContactSteps
            << " all_active_contact_steps=" << first.allActiveContactSteps
            << " max_surface_pose_delta=" << first.maximumSurfacePoseDelta
            << " adaptive_position_checksum="
            << first.finalAdaptivePositionChecksum
            << " all_active_position_checksum="
            << first.finalAllActivePositionChecksum
            << " repeat_deterministic=true\n";
}
