/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <gtest/gtest.h>

#include "dart/dart.hpp"
#include "dart/utils/sdf/SdfParser.hpp"

#include "TestHelpers.hpp"

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

//==============================================================================
TEST(Issue1624, ContactGrouping)
{
  // Load a world with a large number of contacts and run simulation to ensure
  // constraint solver does not misbehave.
  auto world
      = SdfParser::readWorld("dart://sample/sdf/test/issue1624_cubes.sdf");
  ASSERT_TRUE(world != nullptr);
  world->getConstraintSolver()->setCollisionDetector(
      OdeCollisionDetector::create());
  const double dt = 0.001;
  world->setTimeStep(dt);
  std::size_t maxSteps = 1000;
  for (std::size_t i = 0; i < maxSteps; i++)
  {
    world->step();
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    auto skel = world->getSkeleton(i);
    Eigen::Vector3d velocity = skel->getCOMLinearVelocity();
    EXPECT_LE(velocity.norm(), 2.0);
  }
}
