/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <iostream>
#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/experimental/experimental.hpp>
#include <test/TestHelpers.hpp>

using namespace dart;

void setRandomState(const dynamics::SkeletonPtr& skel);

dynamics::SkeletonPtr createRandomSkeleton();
dynamics::SkeletonPtr createRandomSkeletonWithBallJoints();

//==============================================================================
TEST(VariationalIntegrator, SkeletonViRiqnDrnea)
{
  auto skel = createRandomSkeleton();
  auto vi = skel->createAspect<dynamics::SkeletonViRiqnDrnea>();

  const auto numDofs = skel->getNumDofs();
  Eigen::VectorXd randomPos = Eigen::VectorXd::Random(numDofs);

  vi->evaluateDelDeriv(randomPos);
}

//==============================================================================
TEST(VariationalIntegrator, Basic)
{
#ifdef NDEBUG
  auto numSteps = 1e+3;
#else
  auto numSteps = 1e+2;
#endif

  auto skel = createRandomSkeleton();
  auto vi = skel->createAspect<dynamics::SkeletonViRiqnDrnea>();

  for (auto i = 0u; i < numSteps; ++i)
    vi->integrate();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//==============================================================================
//
//                              Implementations
//
//==============================================================================

//==============================================================================
void setRandomState(const dynamics::SkeletonPtr& skel)
{
  const auto pi = math::constantsd::pi();
  const auto numDofs = skel->getNumDofs();
  const auto posLower = pi * -0.5;
  const auto posUpper = pi *  0.5;
  const auto velLower = pi * -0.5;
  const auto velUpper = pi *  0.5;

  for (auto i = 0u; i < numDofs; ++i)
  {
    auto dof = skel->getDof(i);

    const auto pos = math::random(posLower, posUpper);
    const auto vel = math::random(velLower, velUpper);

    dof->setPosition(pos);
    dof->setVelocity(vel);
  }
}

//==============================================================================
SkeletonPtr createRandomSkeleton()
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  auto skel = createNLinkRobot(numLinks, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}

//==============================================================================
SkeletonPtr createRandomSkeletonWithBallJoints()
{
  const auto numLinks = 25u;
  const auto l = 1.5;
  auto skel = createNLinkRobot(numLinks, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}
