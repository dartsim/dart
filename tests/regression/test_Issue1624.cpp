/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/dart.hpp"
#include "dart/io/sdf/SdfParser.hpp"

#include <dart/test/io/TestHelpers.hpp>

#include <dart/dynamics/ode/OdeCollisionDetector.hpp>

#include <gtest/gtest.h>

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::io;

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
#if defined(NDEBUG)
  std::size_t maxSteps = 1000;
#else
  std::size_t maxSteps = 100;
#endif
  for (std::size_t i = 0; i < maxSteps; i++) {
    world->step();
  }

  world->eachSkeleton([&](Skeleton* skel) {
    math::Vector3d velocity = skel->getCOMLinearVelocity();
    EXPECT_LE(velocity.norm(), 2.0);
  });
}
