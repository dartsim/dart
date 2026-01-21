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

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;

//==============================================================================
SkeletonPtr createSimpleSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, RevoluteJoint::Properties(), BodyNode::AspectProperties("body"));
  pair.first->setName("joint");

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  return skel;
}

//==============================================================================
TEST(WorldTests, CreateDefault)
{
  auto world = World::create();

  ASSERT_NE(world, nullptr);
  EXPECT_EQ(world->getName(), "world");
  EXPECT_EQ(world->getNumSkeletons(), 0u);
  EXPECT_EQ(world->getTime(), 0.0);
}

//==============================================================================
TEST(WorldTests, CreateWithName)
{
  auto world = World::create("my_world");

  EXPECT_EQ(world->getName(), "my_world");
}

//==============================================================================
TEST(WorldTests, CreateWithConfig)
{
  WorldConfig config;
  config.name = "configured_world";
  config.collisionDetector = CollisionDetectorType::Dart;

  auto world = World::create(config);

  EXPECT_EQ(world->getName(), "configured_world");
}

//==============================================================================
TEST(WorldTests, SetName)
{
  auto world = World::create("original");

  world->setName("renamed");
  EXPECT_EQ(world->getName(), "renamed");
}

//==============================================================================
TEST(WorldTests, Gravity)
{
  auto world = World::create();

  Eigen::Vector3d defaultGravity = world->getGravity();
  EXPECT_NEAR(defaultGravity[2], -9.81, 0.01);

  world->setGravity(Eigen::Vector3d(0, 0, -10.0));
  Eigen::Vector3d newGravity = world->getGravity();
  EXPECT_DOUBLE_EQ(newGravity[2], -10.0);

  world->setGravity(1.0, 2.0, -5.0);
  Eigen::Vector3d vectorGravity = world->getGravity();
  EXPECT_DOUBLE_EQ(vectorGravity[0], 1.0);
  EXPECT_DOUBLE_EQ(vectorGravity[1], 2.0);
  EXPECT_DOUBLE_EQ(vectorGravity[2], -5.0);
}

//==============================================================================
TEST(WorldTests, TimeStep)
{
  auto world = World::create();

  double defaultTimeStep = world->getTimeStep();
  EXPECT_GT(defaultTimeStep, 0.0);

  world->setTimeStep(0.002);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.002);

  world->setTimeStep(-0.001);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.002);

  world->setTimeStep(0.0);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.002);
}

//==============================================================================
TEST(WorldTests, AddSkeleton)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("test_skel");

  EXPECT_EQ(world->getNumSkeletons(), 0u);

  world->addSkeleton(skel);

  EXPECT_EQ(world->getNumSkeletons(), 1u);
  EXPECT_TRUE(world->hasSkeleton(skel));
  EXPECT_TRUE(world->hasSkeleton("test_skel"));
}

//==============================================================================
TEST(WorldTests, RemoveSkeleton)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("test_skel");

  world->addSkeleton(skel);
  EXPECT_EQ(world->getNumSkeletons(), 1u);

  world->removeSkeleton(skel);
  EXPECT_EQ(world->getNumSkeletons(), 0u);
  EXPECT_FALSE(world->hasSkeleton(skel));
}

//==============================================================================
TEST(WorldTests, RemoveAllSkeletons)
{
  auto world = World::create();

  auto skel1 = createSimpleSkeleton("skel1");
  auto skel2 = createSimpleSkeleton("skel2");
  auto skel3 = createSimpleSkeleton("skel3");

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);
  world->addSkeleton(skel3);

  EXPECT_EQ(world->getNumSkeletons(), 3u);

  auto removed = world->removeAllSkeletons();

  EXPECT_EQ(world->getNumSkeletons(), 0u);
  EXPECT_EQ(removed.size(), 3u);
  EXPECT_TRUE(removed.count(skel1) > 0);
  EXPECT_TRUE(removed.count(skel2) > 0);
  EXPECT_TRUE(removed.count(skel3) > 0);
}

//==============================================================================
TEST(WorldTests, GetSkeletonByIndex)
{
  auto world = World::create();

  auto skel1 = createSimpleSkeleton("skel1");
  auto skel2 = createSimpleSkeleton("skel2");

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  EXPECT_EQ(world->getSkeleton(0), skel1);
  EXPECT_EQ(world->getSkeleton(1), skel2);
  EXPECT_EQ(world->getSkeleton(999), nullptr);
}

//==============================================================================
TEST(WorldTests, GetSkeletonByName)
{
  auto world = World::create();

  auto skel = createSimpleSkeleton("findme");
  world->addSkeleton(skel);

  EXPECT_EQ(world->getSkeleton("findme"), skel);
  EXPECT_EQ(world->getSkeleton("nonexistent"), nullptr);
}

//==============================================================================
TEST(WorldTests, SimpleFrame)
{
  auto world = World::create();

  auto frame = SimpleFrame::createShared(Frame::World(), "test_frame");

  world->addSimpleFrame(frame);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);

  auto retrieved = world->getSimpleFrame(0);
  EXPECT_EQ(retrieved, frame);

  auto retrievedByName = world->getSimpleFrame("test_frame");
  EXPECT_EQ(retrievedByName, frame);

  world->removeSimpleFrame(frame);
  EXPECT_EQ(world->getNumSimpleFrames(), 0u);
}

//==============================================================================
TEST(WorldTests, RemoveAllSimpleFrames)
{
  auto world = World::create();

  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");

  world->addSimpleFrame(frame1);
  world->addSimpleFrame(frame2);

  EXPECT_EQ(world->getNumSimpleFrames(), 2u);

  auto removed = world->removeAllSimpleFrames();

  EXPECT_EQ(world->getNumSimpleFrames(), 0u);
  EXPECT_EQ(removed.size(), 2u);
}

//==============================================================================
TEST(WorldTests, TimeManagement)
{
  auto world = World::create();

  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);

  world->setTime(1.5);
  EXPECT_DOUBLE_EQ(world->getTime(), 1.5);

  world->reset();
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
}

//==============================================================================
TEST(WorldTests, StepCounter)
{
  auto world = World::create();

  EXPECT_EQ(world->getSimFrames(), 0);

  world->step();
  EXPECT_EQ(world->getSimFrames(), 1);

  world->step();
  EXPECT_EQ(world->getSimFrames(), 2);

  world->reset();
  EXPECT_EQ(world->getSimFrames(), 0);
}

//==============================================================================
TEST(WorldTests, BasicSimulation)
{
  auto world = World::create();
  world->setTimeStep(0.001);

  auto skel = Skeleton::create("falling_body");

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  world->addSkeleton(skel);

  double initialZ = skel->getPositions()[5];

  for (int i = 0; i < 100; ++i) {
    world->step();
  }

  double finalZ = skel->getPositions()[5];

  EXPECT_LT(finalZ, initialZ);
}

//==============================================================================
TEST(WorldTests, Clone)
{
  auto world = World::create("original_world");
  world->setGravity(0, 0, -5.0);
  world->setTimeStep(0.002);

  auto skel = createSimpleSkeleton("skel");
  world->addSkeleton(skel);

  auto clone = world->clone();

  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "original_world");
  EXPECT_EQ(clone->getNumSkeletons(), 1u);
  EXPECT_DOUBLE_EQ(clone->getTimeStep(), 0.002);

  Eigen::Vector3d gravity = clone->getGravity();
  EXPECT_DOUBLE_EQ(gravity[2], -5.0);
}

//==============================================================================
TEST(WorldTests, ConstraintSolver)
{
  auto world = World::create();

  auto solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
}

//==============================================================================
TEST(WorldTests, CollisionDetector)
{
  auto world = World::create();

  auto detector = world->getCollisionDetector();
  ASSERT_NE(detector, nullptr);
}

//==============================================================================
TEST(WorldTests, SetCollisionDetector)
{
  auto world = World::create();

  auto detector = world->getCollisionDetector();
  ASSERT_NE(detector, nullptr);

  world->setCollisionDetector(CollisionDetectorType::Dart);
  auto dartDetector = world->getCollisionDetector();
  ASSERT_NE(dartDetector, nullptr);

  world->setCollisionDetector(CollisionDetectorType::Fcl);
  auto fclDetector = world->getCollisionDetector();
  ASSERT_NE(fclDetector, nullptr);
}

//==============================================================================
TEST(WorldTests, GetIndex)
{
  auto world = World::create();

  auto skel1 = createSimpleSkeleton("skel1");
  auto skel2 = createSimpleSkeleton("skel2");

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  EXPECT_EQ(world->getIndex(0), 0);
  EXPECT_EQ(world->getIndex(1), 1);
  EXPECT_EQ(world->getIndex(2), 2);
}

//==============================================================================
TEST(WorldTests, Recording)
{
  auto world = World::create();

  auto recording = world->getRecording();
  EXPECT_NE(recording, nullptr);
}

//==============================================================================
TEST(WorldTests, CheckCollision)
{
  auto world = World::create();

  bool collision = world->checkCollision();
  EXPECT_FALSE(collision);

  auto& lastResult = world->getLastCollisionResult();
  (void)lastResult;
}

//==============================================================================
TEST(WorldTests, SensorManager)
{
  auto world = World::create();

  auto& sensorManager = world->getSensorManager();
  (void)sensorManager;
}

//==============================================================================
TEST(WorldTests, MultipleSkeletonsWithCollision)
{
  auto world = World::create();

  auto skel1 = Skeleton::create("box1");
  auto pair1 = skel1->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body1"));

  auto shape1 = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair1.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape1);

  Inertia inertia1;
  inertia1.setMass(1.0);
  pair1.second->setInertia(inertia1);

  auto skel2 = Skeleton::create("box2");
  auto pair2 = skel2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body2"));

  auto shape2 = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair2.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape2);

  Inertia inertia2;
  inertia2.setMass(1.0);
  pair2.second->setInertia(inertia2);

  Eigen::Vector6d pos2 = Eigen::Vector6d::Zero();
  pos2[5] = 10.0;
  skel2->setPositions(pos2);

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  bool collision = world->checkCollision();
  EXPECT_FALSE(collision);
}

//==============================================================================
TEST(WorldTests, EmptyWorldStep)
{
  auto world = World::create();

  world->step();
  EXPECT_GT(world->getTime(), 0.0);
  EXPECT_EQ(world->getSimFrames(), 1);
}
