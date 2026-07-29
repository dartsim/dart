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

#include "examples/demos/scenes/SoftWormModel.hpp"

#include <gtest/gtest.h>

#include <iomanip>
#include <iostream>

#include <cstddef>

namespace {

constexpr std::size_t kVerificationSteps = 3000;
constexpr double kMinimumDisplacement = 0.2;

struct Outcome
{
  bool finite = true;
  std::size_t nonFiniteStep = 0;
  double displacement = 0.0;
  double checksum = 0.0;
};

Outcome runWorm()
{
  dart::dynamics::SkeletonPtr worm;
  const auto world = dart_demos::soft_worm_model::createWorld(worm);
  world->setNumSimulationThreads(1);
  const double initialRootX = dart_demos::soft_worm_model::getRootX(worm);

  Outcome outcome;
  for (std::size_t step = 1; step <= kVerificationSteps; ++step) {
    dart_demos::soft_worm_model::applyGait(worm, world->getTime(), true);
    world->step();
    if (!dart_demos::soft_worm_model::isFinite(worm)) {
      outcome.finite = false;
      outcome.nonFiniteStep = step;
      return outcome;
    }
  }

  outcome.displacement
      = dart_demos::soft_worm_model::getRootX(worm) - initialRootX;
  outcome.checksum = dart_demos::soft_worm_model::positionChecksum(worm);
  return outcome;
}

} // namespace

//==============================================================================
TEST(SoftWormModelTest, RepeatsFiniteLocomotionDeterministically)
{
  const Outcome first = runWorm();
  ASSERT_TRUE(first.finite)
      << "first run became non-finite at step " << first.nonFiniteStep;
  ASSERT_GT(first.displacement, kMinimumDisplacement);

  const Outcome second = runWorm();
  ASSERT_TRUE(second.finite)
      << "second run became non-finite at step " << second.nonFiniteStep;
  ASSERT_GT(second.displacement, kMinimumDisplacement);
  ASSERT_EQ(second.displacement, first.displacement);
  ASSERT_EQ(second.checksum, first.checksum);

  std::cout << std::setprecision(17) << "soft_worm steps=" << kVerificationSteps
            << " displacement=" << first.displacement
            << " position_checksum=" << first.checksum
            << " repeat_deterministic=true\n";
}
