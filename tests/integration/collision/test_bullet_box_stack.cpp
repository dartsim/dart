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

#include "helpers/dynamics_helpers.hpp"

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/bullet/bullet_collision_detector.hpp>

#include <dart/dynamics/skeleton.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

using namespace dart;
using dart::simulation::World;

namespace {

constexpr double kBoxSide = 0.4;
constexpr double kBoxHeight = 0.5;
constexpr std::size_t kNumBoxes = 10;
constexpr double kTimeStep = 0.001;
constexpr int kNumSteps = 2000;

//==============================================================================
std::vector<dynamics::SkeletonPtr> createStack(World& world)
{
  // Ground
  auto ground = createBox({5.0, 5.0, 0.4}, {0.0, 0.0, -0.2});
  ground->setMobile(false);
  world.addSkeleton(ground);

  // Boxes
  std::vector<dynamics::SkeletonPtr> boxes;
  boxes.reserve(kNumBoxes);

  for (std::size_t i = 0; i < kNumBoxes; ++i) {
    const double centerZ
        = kBoxHeight * 0.5 + static_cast<double>(i) * kBoxHeight;
    auto box = createBox({kBoxSide, kBoxSide, kBoxHeight}, {0.0, 0.0, centerZ});
    world.addSkeleton(box);
    boxes.push_back(std::move(box));
  }

  return boxes;
}

} // namespace

//==============================================================================
TEST(Issue867, BulletBoxStackingStaysStable)
{
  auto world = World::create();
  world->setTimeStep(kTimeStep);

  auto lcpSolver = std::make_shared<math::DantzigSolver>();
  auto solver = std::make_unique<constraint::ConstraintSolver>(lcpSolver);
  solver->setCollisionDetector(collision::BulletCollisionDetector::create());
  world->setConstraintSolver(std::move(solver));

  auto boxes = createStack(*world);
  ASSERT_EQ(boxes.size(), kNumBoxes);

  const double expectedTopCenter
      = kBoxHeight * (static_cast<double>(kNumBoxes) - 0.5);
  double maxObservedTopZ = -std::numeric_limits<double>::infinity();
  double finalKineticEnergy = 0.0;

  for (int step = 0; step < kNumSteps; ++step) {
    world->step();

    // Ensure bodies remain finite to catch explosions/NANs early.
    for (const auto& box : boxes) {
      const auto pos = box->getBodyNode(0)->getWorldTransform().translation();
      ASSERT_TRUE(pos.array().allFinite());
    }

    const auto topPos
        = boxes.back()->getBodyNode(0)->getWorldTransform().translation();
    maxObservedTopZ = std::max(maxObservedTopZ, topPos.z());

    if (step == kNumSteps - 1) {
      for (const auto& box : boxes)
        finalKineticEnergy += box->computeKineticEnergy();
    }
  }

  EXPECT_LT(maxObservedTopZ, expectedTopCenter + 2.0);
  EXPECT_GT(
      boxes.back()->getBodyNode(0)->getWorldTransform().translation().z(),
      expectedTopCenter - 0.5);
  EXPECT_LT(finalKineticEnergy, 5.0);
}
