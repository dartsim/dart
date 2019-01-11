/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/math/Geometry.hpp"
#include "dart/io/SkelParser.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/collision/collision.hpp"
#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

//==============================================================================
TEST(World, AddingAndRemovingSkeletons)
{
  // World
  WorldPtr world = World::create();

  //-------------------- Test World::removeSkeleton() ------------------------
  SkeletonPtr skeleton1 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

  SkeletonPtr skeleton2 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

  SkeletonPtr skeleton3 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

  SkeletonPtr skeleton4 = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_X,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Y,
                                               Eigen::Vector3d(1.0, 1.0, 1.0),
                                               DOF_Z,
                                               false, false);

  int nSteps = 20;

  // Empty world
  for (int i = 0; i < nSteps; ++i)
      world->step();

  EXPECT_FALSE(world->hasSkeleton(skeleton1));
  EXPECT_FALSE(world->hasSkeleton(skeleton2));
  EXPECT_FALSE(world->hasSkeleton(skeleton3));
  EXPECT_FALSE(world->hasSkeleton(skeleton4));

  // Add skeleton1, skeleton2
  world->addSkeleton(skeleton1);
  EXPECT_TRUE(world->hasSkeleton(skeleton1));
  world->addSkeleton(skeleton2);
  EXPECT_TRUE(world->hasSkeleton(skeleton2));
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
      world->step();

  std::string s1name = skeleton1->getName();
  std::string s2name = skeleton2->getName();
  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(skeleton2 == world->getSkeleton(s2name));

  // Remove skeleton2
  world->removeSkeleton(skeleton2);
  EXPECT_TRUE(world->getNumSkeletons() == 1);
  for (int i = 0; i < nSteps; ++i)
      world->step();

  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_FALSE(skeleton2 == world->getSkeleton(s2name));
  EXPECT_TRUE(world->getSkeleton(s2name) == nullptr);

  // Add skeleton3, skeleton4
  world->addSkeleton(skeleton3);
  EXPECT_TRUE(world->hasSkeleton(skeleton3));
  world->addSkeleton(skeleton4);
  EXPECT_TRUE(world->hasSkeleton(skeleton4));
  EXPECT_TRUE(world->getNumSkeletons() == 3);
  for (int i = 0; i < nSteps; ++i)
      world->step();

  std::string s3name = skeleton3->getName();
  std::string s4name = skeleton4->getName();

  EXPECT_TRUE(s3name == s2name);
  EXPECT_TRUE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(skeleton4 == world->getSkeleton(s4name));

  skeleton4->setName(skeleton1->getName());
  EXPECT_FALSE(skeleton4->getName() == skeleton1->getName());

  // Remove skeleton1
  world->removeSkeleton(skeleton1);
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
      world->step();

  EXPECT_FALSE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(world->getSkeleton(s1name) == nullptr);

  // Remove all the skeletons
  world->removeAllSkeletons();
  EXPECT_EQ((int)world->getNumSkeletons(), 0);
  for (int i = 0; i < nSteps; ++i)
      world->step();

  EXPECT_FALSE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(world->getSkeleton(s3name) == nullptr);

  EXPECT_FALSE(skeleton4 == world->getSkeleton(s4name));
  EXPECT_TRUE(world->getSkeleton(s4name) == nullptr);

  // An error will be thrown here if Skeletons are not being removed correctly
  skeleton1->setName(skeleton4->getName());
}

//==============================================================================
TEST(World, Cloning)
{
  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for(std::size_t i=0; i<fileList.size(); ++i)
    worlds.push_back(io::SkelParser::readWorld(fileList[i]));

  for(std::size_t i=0; i<worlds.size(); ++i)
  {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for(std::size_t j=1; j<5; ++j)
      clones.push_back(clones[j-1]->clone());

#ifndef NDEBUG // Debug mode
    std::size_t numIterations = 3;
#else
    std::size_t numIterations = 500;
#endif

    for(std::size_t j=0; j<numIterations; ++j)
    {
      for(std::size_t k=0; k<original->getNumSkeletons(); ++k)
      {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);

        // Generate a random command vector
        Eigen::VectorXd commands = skel->getCommands();
        for(int q=0; q<commands.size(); ++q)
          commands[q] = Random::uniform(-0.1, 0.1);

        // Assign the command vector to each clone of the kth skeleton
        for(std::size_t c=0; c<clones.size(); ++c)
        {
          dart::dynamics::SkeletonPtr skelClone = clones[c]->getSkeleton(k);
          skelClone->setCommands(commands);
        }
      }

      // Step each clone forward
      for(std::size_t c=0; c<clones.size(); ++c)
        clones[c]->step(false);
    }

    for(std::size_t c=0; c<clones.size(); ++c)
    {
      for(std::size_t k=0; k<original->getNumSkeletons(); ++k)
      {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);
        dart::dynamics::SkeletonPtr clone = clones[c]->getSkeleton(k);

        EXPECT_TRUE( equals(skel->getPositions(), clone->getPositions(), 0));
        EXPECT_TRUE( equals(skel->getVelocities(), clone->getVelocities(), 0));
        EXPECT_TRUE( equals(skel->getAccelerations(), clone->getAccelerations(), 0));
        EXPECT_TRUE( equals(skel->getForces(), clone->getForces(), 0));
      }
    }
  }
}

//==============================================================================
TEST(World, ValidatingClones)
{
  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for(std::size_t i=0; i<fileList.size(); ++i)
  {
    worlds.push_back(io::SkelParser::readWorld(fileList[i]));

    // Set non default collision detector
#if HAVE_BULLET
    worlds.back()->getConstraintSolver()->setCollisionDetector(
        collision::BulletCollisionDetector::create());
#else
    worlds.back()->getConstraintSolver()->setCollisionDetector(
        collision::DARTCollisionDetector::create());
#endif
  }

  for(std::size_t i=0; i<worlds.size(); ++i)
  {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for(std::size_t j=1; j<5; ++j)
    {
      clones.push_back(clones[j-1]->clone());

      auto originalCD
          = original->getConstraintSolver()->getCollisionDetector();
      auto cloneCD
          = clones.back()->getConstraintSolver()->getCollisionDetector();

      std::string originalCDType = originalCD->getType();
      std::string cloneCDType = cloneCD->getType();

      EXPECT_EQ(originalCDType, cloneCDType);
    }
  }
}
