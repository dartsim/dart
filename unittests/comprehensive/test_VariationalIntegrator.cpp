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

dynamics::SkeletonPtr createRandomSkeleton(std::size_t numBodies = 25u);
dynamics::SkeletonPtr createRandomSkeletonWithBallJoints(std::size_t numBodies = 25u);

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
template <typename Integrator>
void testEnergyBehavior()
{
#ifdef NDEBUG
  auto numSteps = 1e+4;
#else
  auto numSteps = 1e+1;
#endif

  auto skel = createRandomSkeleton(25u);
  ////
  skel->setGravity(Eigen::Vector3d::Zero());
  ////
  auto vi = skel->createAspect<Integrator>();
  auto E0 = skel->computeTotalEnergy();

//  skel->setMaxIternation(3u);

  for (auto i = 0u; i < numSteps; ++i)
    skel->integrate();

  auto E1 = skel->computeTotalEnergy();
  auto errorInPercentage = (E1 - E0)/E0 * 100.0;

  std::cout << "E0: " << E0 << std::endl;
  std::cout << "E1: " << E1 << std::endl;
  std::cout << "E(%): " << errorInPercentage << std::endl;

  // The error should be less than 5.0%
  EXPECT_LT(errorInPercentage, 5.0);
}

//==============================================================================
TEST(VariationalIntegrator, EnergyBehavior)
{
  common::Timer t;

//  t.start();
//  testEnergyBehavior<dynamics::SkeletonViRiqnSvi>();
//  t.stop();
//  std::cout << "RIQN + SVI  : " << t.getLastElapsedTime() << " (sec)" << std::endl;

  t.start();
  testEnergyBehavior<dynamics::SkeletonViRiqnDrnea>();
  t.stop();
  std::cout << "RIQN + DRNEA: " << t.getLastElapsedTime() << " (sec)" << std::endl;
}

//==============================================================================
TEST(VariationalIntegrator, CompareVariationalIntegrators)
{
#ifdef NDEBUG
  auto numSteps = 1e+3;
#else
  auto numSteps = 1e+1;
#endif

  const auto numBodies = 2u;

  auto skel = createRandomSkeleton(numBodies);
  ////
  skel->setGravity(Eigen::Vector3d::Zero());
  ////

  auto skelSvi = skel->clone();
  auto skelDrnea = skel->clone();

  auto E0Svi = skelSvi->computeTotalEnergy();
  auto E0Drnea = skelDrnea->computeTotalEnergy();
  EXPECT_NEAR(E0Svi, E0Drnea, 1e-9);

  auto viSvi = skelSvi->createAspect<dynamics::SkeletonViRiqnSvi>();
  auto viDrnea = skelDrnea->createAspect<dynamics::SkeletonViRiqnDrnea>();

  Eigen::VectorXd posSvi = skelSvi->getPositions();
  Eigen::VectorXd posDrnea = skelDrnea->getPositions();

  Eigen::VectorXd velSvi = skelSvi->getVelocities();
  Eigen::VectorXd velDrnea = skelDrnea->getVelocities();

  Eigen::VectorXd randomPositions = Eigen::VectorXd::Random(numBodies);
  Eigen::VectorXd randomVelocities = Eigen::VectorXd::Random(numBodies);

  skelSvi->setPositions(randomPositions);
  skelDrnea->setPositions(randomPositions);

  skelSvi->setVelocities(randomVelocities);
  skelDrnea->setVelocities(randomVelocities);

  const Eigen::VectorXd positions = skelSvi->getPositions();

  Eigen::VectorXd delSvi = viSvi->evaluateDel(positions);
  Eigen::VectorXd delDrnea = viDrnea->evaluateDel(positions);

  EXPECT_TRUE(delSvi.isApprox(delDrnea, 1e-4));
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
SkeletonPtr createRandomSkeleton(std::size_t numBodies)
{
  const auto l = 1.5;
  auto skel = createNLinkRobot(numBodies, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}

//==============================================================================
SkeletonPtr createRandomSkeletonWithBallJoints(std::size_t numBodies)
{
  const auto l = 1.5;
  auto skel = createNLinkRobot(numBodies, Eigen::Vector3d(0.3, 0.3, l), DOF_ROLL);
  setRandomState(skel);

  return skel;
}
