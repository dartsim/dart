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

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.hpp"

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/io/SkelParser.hpp"

//==============================================================================
class ConstraintTest : public ::testing::Test
{
public:
  // Get Skel file list to test.
  const std::vector<std::string>& getList();

  //
  void SingleContactTest(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  void SetUp() override;

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void ConstraintTest::SetUp()
{
  list.push_back("dart://sample/skel/test/chainwhipa.skel");
  list.push_back("dart://sample/skel/test/single_pendulum.skel");
  list.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  list.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  list.push_back("dart://sample/skel/test/double_pendulum.skel");
  list.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  list.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  list.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  list.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  list.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  list.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  list.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  list.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  list.push_back("dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  list.push_back("dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  list.push_back("dart://sample/skel/test/tree_structure.skel");
  list.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  list.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  list.push_back("dart://sample/skel/fullbody1.skel");
}

//==============================================================================
const std::vector<std::string>& ConstraintTest::getList()
{
  return list;
}

//==============================================================================
void ConstraintTest::SingleContactTest(const std::string& /*_fileName*/)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart::math;
  using namespace dart::collision;
  using namespace dart::constraint;
  using namespace dart::dynamics;
  using namespace dart::simulation;
  using namespace dart::io;

  //----------------------------------------------------------------------------
  // Settings
  //----------------------------------------------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  // std::size_t testCount = 1;
#else
  // std::size_t testCount = 1;
#endif

  WorldPtr world = World::create();
  EXPECT_TRUE(world != nullptr);
  world->setGravity(Vector3d(0.0, -10.00, 0.0));
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(
        DARTCollisionDetector::create());

  SkeletonPtr sphereSkel = createSphere(0.05, Vector3d(0.0, 1.0, 0.0));
  BodyNode* sphere = sphereSkel->getBodyNode(0);
  Joint* sphereJoint = sphere->getParentJoint();
  sphereJoint->setVelocity(3, Random::uniform(-2.0, 2.0));  // x-axis
  sphereJoint->setVelocity(5, Random::uniform(-2.0, 2.0));  // z-axis
  world->addSkeleton(sphereSkel);
  EXPECT_EQ(sphereSkel->getGravity(), world->getGravity());
  assert(sphere);

  SkeletonPtr boxSkel = createBox(Vector3d(1.0, 1.0, 1.0),
                                  Vector3d(0.0, 1.0, 0.0));
  BodyNode* box = boxSkel->getBodyNode(0);
  Joint* boxJoint = box->getParentJoint();
  boxJoint->setVelocity(3, Random::uniform(-2.0, 2.0));  // x-axis
  boxJoint->setVelocity(5, Random::uniform(-2.0, 2.0));  // z-axis
//  world->addSkeleton(boxSkel);
//  EXPECT_EQ(boxSkel->getGravity(), world->getGravity());
//  assert(box);

  SkeletonPtr groundSkel = createGround(Vector3d(10000.0, 0.1, 10000.0),
                                      Vector3d(0.0, -0.05, 0.0));
  groundSkel->setMobile(false);
  // BodyNode* ground = groundSkel->getBodyNode(0);
  world->addSkeleton(groundSkel);
  EXPECT_EQ(groundSkel->getGravity(), world->getGravity());
  // assert(ground);

  EXPECT_EQ((int)world->getNumSkeletons(), 2);

  // Lower and upper bound of configuration for system
  // double lb = -1.5 * constantsd::pi();
  // double ub =  1.5 * constantsd::pi();

  int maxSteps = 500;
  for (int i = 0; i < maxSteps; ++i)
  {
//    Vector3d pos1 = sphere->getWorldTransform().translation();
//    Vector3d vel1 = sphere->getWorldLinearVelocity(pos1);

//    std::cout << "pos1:" << pos1.transpose() << std::endl;
//    std::cout << "vel1:" << vel1.transpose() << std::endl;

    if (!world->checkCollision())
    {
      world->step();
      continue;
    }

    // for (std::size_t j = 0; j < cd->getNumContacts(); ++j)
    // {
      // Contact contact = cd->getContact(j);
      // Vector3d pos1 = sphere->getTransform().inverse() * contact.point;
      // Vector3d vel1 = sphere->getWorldLinearVelocity(pos1);

      // std::cout << "pos1:" << pos1.transpose() << std::endl;
      // std::cout << "vel1:" << vel1.transpose() << std::endl;
    // }

    world->step();

    const auto& result = world->getConstraintSolver()->getLastCollisionResult();

    for (const auto& contact : result.getContacts())
    {
      Vector3d pos1 = sphere->getTransform().inverse() * contact.point;
      Vector3d vel1 = sphere->getLinearVelocity(pos1);

//      std::cout << "pos1:" << pos1.transpose() << std::endl;

//      std::cout << "pos1[1]: " << pos1[1] << std::endl;

//      std::cout << "pos1:" << pos1.transpose() << std::endl;
      std::cout << "vel1:" << vel1.transpose() << ", pos1[1]: " << pos1[1] << std::endl;

//      EXPECT_NEAR(pos1[0], 0.0, 1e-9);
//      EXPECT_NEAR(pos1[1], -0.05, 1e-2);
//      EXPECT_NEAR(pos1[2], 0.0, 1e-9);

//      EXPECT_NEAR(vel1[0], 0.0, 1e-9);
//      EXPECT_NEAR(vel1[1], 0.0, 1e-9);
//      EXPECT_NEAR(vel1[2], 0.0, 1e-9);

//      if (!equals(vel1, Vector3d(0.0, 0.0, 0.0)))
//        std::cout << "vel1:" << vel1.transpose() << std::endl;

//      EXPECT_EQ(vel1, Vector3d::Zero());
    }

//    std::cout << std::endl;

    break;
  }
}

//==============================================================================
TEST_F(ConstraintTest, SingleContactTest)
{
//  for (int i = 0; i < getList().size(); ++i)
//  {
//#ifndef NDEBUG
//    dtdbg << getList()[i] << std::endl;
//#endif
//    SingleContactTest(getList()[i]);
//  }

  SingleContactTest(getList()[0]);
}
