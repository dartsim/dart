/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"

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
  virtual void SetUp();

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void ConstraintTest::SetUp()
{
  list.push_back(KIDO_DATA_PATH"skel/test/chainwhipa.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/single_pendulum.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/single_pendulum_euler_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/single_pendulum_ball_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/double_pendulum.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/double_pendulum_euler_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/double_pendulum_ball_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/serial_chain_revolute_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/serial_chain_ball_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/serial_chain_ball_joint_20.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/serial_chain_ball_joint_40.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/simple_tree_structure.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/simple_tree_structure_ball_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/tree_structure.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/tree_structure_euler_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/test/tree_structure_ball_joint.skel");
  list.push_back(KIDO_DATA_PATH"skel/fullbody1.skel");
}

//==============================================================================
const std::vector<std::string>& ConstraintTest::getList()
{
  return list;
}

//==============================================================================
void ConstraintTest::SingleContactTest(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace kido::math;
  using namespace kido::collision;
  using namespace kido::constraint;
  using namespace kido::dynamics;
  using namespace kido::simulation;
  using namespace kido::utils;

  //----------------------------------------------------------------------------
  // Settings
  //----------------------------------------------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  // size_t testCount = 1;
#else
  // size_t testCount = 1;
#endif

  WorldPtr world(new World);
  EXPECT_TRUE(world != nullptr);
  world->setGravity(Vector3d(0.0, -10.00, 0.0));
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(
        new DARTCollisionDetector());

  SkeletonPtr sphereSkel = createSphere(0.05, Vector3d(0.0, 1.0, 0.0));
  BodyNode* sphere = sphereSkel->getBodyNode(0);
  Joint* sphereJoint = sphere->getParentJoint();
  sphereJoint->setVelocity(3, random(-2.0, 2.0));  // x-axis
  sphereJoint->setVelocity(5, random(-2.0, 2.0));  // z-axis
  world->addSkeleton(sphereSkel);
  EXPECT_EQ(sphereSkel->getGravity(), world->getGravity());
  assert(sphere);

  SkeletonPtr boxSkel = createBox(Vector3d(1.0, 1.0, 1.0),
                                  Vector3d(0.0, 1.0, 0.0));
  BodyNode* box = boxSkel->getBodyNode(0);
  Joint* boxJoint = box->getParentJoint();
  boxJoint->setVelocity(3, random(-2.0, 2.0));  // x-axis
  boxJoint->setVelocity(5, random(-2.0, 2.0));  // z-axis
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

  ConstraintSolver* cs = world->getConstraintSolver();
  CollisionDetector* cd = cs->getCollisionDetector();

  // Lower and upper bound of configuration for system
  // double lb = -1.5 * KIDO_PI;
  // double ub =  1.5 * KIDO_PI;

  int maxSteps = 500;
  for (int i = 0; i < maxSteps; ++i)
  {
//    Vector3d pos1 = sphere->getWorldTransform().translation();
//    Vector3d vel1 = sphere->getWorldLinearVelocity(pos1);

//    std::cout << "pos1:" << pos1.transpose() << std::endl;
//    std::cout << "vel1:" << vel1.transpose() << std::endl;

    cd->detectCollision(true, true);
    if (cd->getNumContacts() == 0)
    {
      world->step();
      continue;
    }

    // for (size_t j = 0; j < cd->getNumContacts(); ++j)
    // {
      // Contact contact = cd->getContact(j);
      // Vector3d pos1 = sphere->getTransform().inverse() * contact.point;
      // Vector3d vel1 = sphere->getWorldLinearVelocity(pos1);

      // std::cout << "pos1:" << pos1.transpose() << std::endl;
      // std::cout << "vel1:" << vel1.transpose() << std::endl;
    // }

    world->step();

    for (size_t j = 0; j < cd->getNumContacts(); ++j)
    {
      Contact contact = cd->getContact(j);
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

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

