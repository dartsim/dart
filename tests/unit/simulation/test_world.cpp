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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

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
std::pair<SkeletonPtr, BodyNode*> createFreeBodySkeleton(
    const std::string& name)
{
  auto skel = Skeleton::create(name);
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.5);
  inertia.setMoment(0.2, 0.25, 0.3, 0.0, 0.0, 0.0);
  pair.second->setInertia(inertia);

  return {skel, pair.second};
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
  EXPECT_TRUE(removed.contains(skel1));
  EXPECT_TRUE(removed.contains(skel2));
  EXPECT_TRUE(removed.contains(skel3));
}

//==============================================================================
TEST(WorldTests, ClonePreservesSkeletonsAndFrames)
{
  auto world = World::create("source_world");
  auto skel = createSimpleSkeleton("clone_skel");
  world->addSkeleton(skel);

  auto frame = SimpleFrame::createShared(Frame::World(), "clone_frame");
  world->addSimpleFrame(frame);

  auto clone = world->clone();
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getNumSkeletons(), 1u);
  EXPECT_EQ(clone->getNumSimpleFrames(), 1u);
  EXPECT_NE(clone->getSkeleton(0), skel);
  EXPECT_NE(clone->getSimpleFrame(0).get(), frame.get());
  EXPECT_EQ(clone->getSkeleton(0)->getName(), "clone_skel");
  EXPECT_EQ(clone->getSimpleFrame(0)->getName(), "clone_frame");
}

//==============================================================================
TEST(WorldTests, RecordingBakesState)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("record_skel");
  world->addSkeleton(skel);

  auto* recording = world->getRecording();
  ASSERT_NE(recording, nullptr);
  recording->clear();
  EXPECT_EQ(recording->getNumFrames(), 0);

  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 1);

  world->step();
  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 2);
}

//==============================================================================
TEST(WorldTests, HandlesNameChanges)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("named_skel");
  world->addSkeleton(skel);

  skel->setName("renamed_skel");
  EXPECT_TRUE(world->hasSkeleton("renamed_skel"));
  EXPECT_FALSE(world->hasSkeleton("named_skel"));

  auto frame = SimpleFrame::createShared(Frame::World(), "named_frame");
  world->addSimpleFrame(frame);

  frame->setName("renamed_frame");
  EXPECT_NE(world->getSimpleFrame("renamed_frame"), nullptr);
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
TEST(WorldTests, SetCollisionDetectorNullKeepsCurrent)
{
  auto world = World::create();

  auto detector = world->getCollisionDetector();
  ASSERT_NE(detector, nullptr);

  world->setCollisionDetector(nullptr);
  EXPECT_EQ(world->getCollisionDetector(), detector);
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
TEST(WorldTests, SetConstraintSolverRejectsNull)
{
  auto world = World::create();

  EXPECT_THROW(
      world->setConstraintSolver(nullptr), dart::common::NullPointerException);
}

//==============================================================================
TEST(WorldTests, SetConstraintSolverCopiesState)
{
  auto world = World::create();
  world->setTimeStep(0.005);
  world->addSkeleton(createSimpleSkeleton("copy_skel"));
  world->getConstraintSolver()->setSplitImpulseEnabled(true);

  auto replacement = std::make_unique<constraint::ConstraintSolver>();
  replacement->setSplitImpulseEnabled(false);
  world->setConstraintSolver(std::move(replacement));

  auto solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);
  EXPECT_TRUE(solver->isSplitImpulseEnabled());
  EXPECT_EQ(solver->getSkeletons().size(), 1u);
  EXPECT_DOUBLE_EQ(solver->getTimeStep(), 0.005);
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

//==============================================================================
TEST(WorldTests, RecordingBasicMethods)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("recording_skel");
  world->addSkeleton(skel);

  auto recording = world->getRecording();
  ASSERT_NE(recording, nullptr);

  EXPECT_EQ(recording->getNumFrames(), 0);
  EXPECT_EQ(recording->getNumSkeletons(), 1);
  EXPECT_EQ(recording->getNumDofs(0), 1);

  world->step();
  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 1);

  world->step();
  world->bake();
  EXPECT_EQ(recording->getNumFrames(), 2);

  Eigen::VectorXd config = recording->getConfig(0, 0);
  EXPECT_EQ(config.size(), 1);

  double genCoord = recording->getGenCoord(0, 0, 0);
  EXPECT_DOUBLE_EQ(genCoord, config[0]);

  recording->clear();
  EXPECT_EQ(recording->getNumFrames(), 0);
}

//==============================================================================
TEST(WorldTests, RecordingAddState)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("state_skel");
  world->addSkeleton(skel);

  auto recording = world->getRecording();

  Eigen::VectorXd state(1);
  state << 0.5;
  recording->addState(state);

  EXPECT_EQ(recording->getNumFrames(), 1);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(0, 0, 0), 0.5);

  state << 1.0;
  recording->addState(state);

  EXPECT_EQ(recording->getNumFrames(), 2);
  EXPECT_DOUBLE_EQ(recording->getGenCoord(1, 0, 0), 1.0);
}

//==============================================================================
TEST(WorldTests, BakeRecordsSimulationState)
{
  auto world = World::create();
  world->setTimeStep(0.001);

  auto skel = Skeleton::create("bake_skel");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  world->addSkeleton(skel);

  auto recording = world->getRecording();
  EXPECT_EQ(recording->getNumFrames(), 0);

  for (int i = 0; i < 10; ++i) {
    world->step();
    world->bake();
  }

  EXPECT_EQ(recording->getNumFrames(), 10);

  Eigen::VectorXd config0 = recording->getConfig(0, 0);
  Eigen::VectorXd config9 = recording->getConfig(9, 0);

  EXPECT_LT(config9[5], config0[5]);
}

//==============================================================================
TEST(WorldTests, SkeletonNameChange)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("original_name");

  world->addSkeleton(skel);
  EXPECT_EQ(world->getSkeleton("original_name"), skel);

  skel->setName("new_name");

  EXPECT_EQ(world->getSkeleton("new_name"), skel);
  EXPECT_EQ(world->getSkeleton("original_name"), nullptr);
}

//==============================================================================
TEST(WorldTests, SimpleFrameNameChange)
{
  auto world = World::create();
  auto frame = SimpleFrame::createShared(Frame::World(), "original_frame");

  world->addSimpleFrame(frame);
  EXPECT_EQ(world->getSimpleFrame("original_frame"), frame);

  frame->setName("renamed_frame");

  EXPECT_EQ(world->getSimpleFrame("renamed_frame"), frame);
  EXPECT_EQ(world->getSimpleFrame("original_frame"), nullptr);
}

//==============================================================================
TEST(WorldTests, StepWithPersistForces)
{
  auto world = World::create();
  world->setTimeStep(0.001);

  auto skel = Skeleton::create("force_skel");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  world->addSkeleton(skel);

  Eigen::Vector3d force(10.0, 0.0, 0.0);
  pair.second->addExtForce(force);

  world->step(false);

  Eigen::Vector6d extForce = pair.second->getExternalForceLocal();
  EXPECT_GT(extForce.norm(), 0.0);

  world->step(true);

  extForce = pair.second->getExternalForceLocal();
  EXPECT_DOUBLE_EQ(extForce.norm(), 0.0);
}

//==============================================================================
TEST(WorldTests, StepSubstepsMatchesManualSmallerTimeStep)
{
  constexpr std::size_t substeps = 10u;
  constexpr double outerTimeStep = 0.01;
  constexpr double internalTimeStep
      = outerTimeStep / static_cast<double>(substeps);

  auto substepWorld = World::create();
  substepWorld->setGravity(Eigen::Vector3d::Zero());
  substepWorld->setTimeStep(outerTimeStep);
  auto [substepSkeleton, substepBody]
      = createFreeBodySkeleton("substep_free_body");
  substepWorld->addSkeleton(substepSkeleton);

  auto manualWorld = World::create();
  manualWorld->setGravity(Eigen::Vector3d::Zero());
  manualWorld->setTimeStep(internalTimeStep);
  auto [manualSkeleton, manualBody]
      = createFreeBodySkeleton("manual_free_body");
  manualWorld->addSkeleton(manualSkeleton);

  Eigen::Vector6d positions;
  positions << 0.05, -0.02, 0.03, 0.1, -0.2, 0.3;
  Eigen::Vector6d velocities;
  velocities << 0.2, -0.1, 0.15, 0.7, -0.3, 0.4;
  substepSkeleton->setPositions(positions);
  manualSkeleton->setPositions(positions);
  substepSkeleton->setVelocities(velocities);
  manualSkeleton->setVelocities(velocities);

  const Eigen::Vector3d force(2.0, -1.0, 0.5);
  const Eigen::Vector3d torque(0.04, -0.02, 0.03);
  substepBody->addExtForce(force);
  manualBody->addExtForce(force);
  substepBody->addExtTorque(torque);
  manualBody->addExtTorque(torque);

  substepWorld->stepSubsteps(substeps);
  for (std::size_t i = 0; i < substeps; ++i) {
    manualWorld->step(i + 1u == substeps);
  }

  EXPECT_EQ(substepWorld->getSimFrames(), manualWorld->getSimFrames());
  EXPECT_NEAR(substepWorld->getTime(), manualWorld->getTime(), 1e-15);
  EXPECT_DOUBLE_EQ(substepWorld->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(substepSkeleton->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(
      substepWorld->getConstraintSolver()->getTimeStep(), outerTimeStep);
  EXPECT_TRUE(substepSkeleton->getPositions().isApprox(
      manualSkeleton->getPositions(), 1e-12))
      << "substepping should match manual smaller-timestep positions";
  EXPECT_TRUE(substepSkeleton->getVelocities().isApprox(
      manualSkeleton->getVelocities(), 1e-12))
      << "substepping should match manual smaller-timestep velocities";
  EXPECT_DOUBLE_EQ(substepBody->getExternalForceLocal().norm(), 0.0);
}

//==============================================================================
TEST(WorldTests, StepSubstepsMatchesManualSmallerTimeStepWithJointCommand)
{
  constexpr std::size_t substeps = 8u;
  constexpr double outerTimeStep = 0.008;
  constexpr double internalTimeStep
      = outerTimeStep / static_cast<double>(substeps);

  auto substepWorld = World::create();
  substepWorld->setGravity(Eigen::Vector3d::Zero());
  substepWorld->setTimeStep(outerTimeStep);
  auto substepSkeleton = createSimpleSkeleton("substep_command");
  auto* substepJoint = substepSkeleton->getJoint(0);
  substepWorld->addSkeleton(substepSkeleton);

  auto manualWorld = World::create();
  manualWorld->setGravity(Eigen::Vector3d::Zero());
  manualWorld->setTimeStep(internalTimeStep);
  auto manualSkeleton = createSimpleSkeleton("manual_command");
  auto* manualJoint = manualSkeleton->getJoint(0);
  manualWorld->addSkeleton(manualSkeleton);

  substepSkeleton->setPosition(0, 0.2);
  manualSkeleton->setPosition(0, 0.2);
  substepSkeleton->setVelocity(0, -0.15);
  manualSkeleton->setVelocity(0, -0.15);

  constexpr double command = 0.7;
  substepJoint->setCommand(0, command);
  manualJoint->setCommand(0, command);

  substepWorld->stepSubsteps(substeps);
  for (std::size_t i = 0; i < substeps; ++i) {
    manualWorld->step(i + 1u == substeps);
  }

  EXPECT_EQ(substepWorld->getSimFrames(), manualWorld->getSimFrames());
  EXPECT_NEAR(substepWorld->getTime(), manualWorld->getTime(), 1e-15);
  EXPECT_DOUBLE_EQ(substepWorld->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(substepSkeleton->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(
      substepWorld->getConstraintSolver()->getTimeStep(), outerTimeStep);
  EXPECT_NEAR(
      substepSkeleton->getPosition(0), manualSkeleton->getPosition(0), 1e-12);
  EXPECT_NEAR(
      substepSkeleton->getVelocity(0), manualSkeleton->getVelocity(0), 1e-12);
  EXPECT_DOUBLE_EQ(substepJoint->getCommand(0), 0.0);
}

//==============================================================================
TEST(WorldTests, StepSubstepsResetsForcesOnlyAfterFinalSubstep)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.004);
  auto [skeleton, body] = createFreeBodySkeleton("persist_force_free_body");
  world->addSkeleton(skeleton);

  body->addExtForce(Eigen::Vector3d(3.0, 0.0, 0.0));
  world->stepSubsteps(4u, false);
  EXPECT_GT(body->getExternalForceLocal().norm(), 0.0);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.004);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.004);
  EXPECT_EQ(world->getSimFrames(), 4);

  world->stepSubsteps(4u, true);
  EXPECT_DOUBLE_EQ(body->getExternalForceLocal().norm(), 0.0);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.004);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.004);
  EXPECT_EQ(world->getSimFrames(), 8);
}

//==============================================================================
TEST(WorldTests, StepSubstepsResetsCommandsOnlyAfterFinalSubstep)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.004);
  auto skeleton = createSimpleSkeleton("persist_command");
  auto* joint = skeleton->getJoint(0);
  world->addSkeleton(skeleton);

  constexpr double command = 0.5;
  joint->setCommand(0, command);
  world->stepSubsteps(4u, false);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), command);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.004);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.004);
  EXPECT_EQ(world->getSimFrames(), 4);

  world->stepSubsteps(4u, true);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);
  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.004);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.004);
  EXPECT_EQ(world->getSimFrames(), 8);
}

//==============================================================================
TEST(WorldTests, StepSubstepsIgnoresZeroSubsteps)
{
  auto world = World::create();
  world->setTimeStep(0.002);
  world->setTime(0.5);

  world->stepSubsteps(0u);

  EXPECT_DOUBLE_EQ(world->getTimeStep(), 0.002);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.5);
  EXPECT_EQ(world->getSimFrames(), 0);
}

//==============================================================================
TEST(WorldTests, AddSkeletonTwice)
{
  auto world = World::create();
  auto skel = createSimpleSkeleton("duplicate_skel");

  world->addSkeleton(skel);
  EXPECT_EQ(world->getNumSkeletons(), 1u);

  world->addSkeleton(skel);
  EXPECT_EQ(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(WorldTests, RemoveNonExistentSkeleton)
{
  auto world = World::create();
  auto skel1 = createSimpleSkeleton("skel1");
  auto skel2 = createSimpleSkeleton("skel2");

  world->addSkeleton(skel1);
  EXPECT_EQ(world->getNumSkeletons(), 1u);

  world->removeSkeleton(skel2);
  EXPECT_EQ(world->getNumSkeletons(), 1u);

  EXPECT_THROW(
      world->removeSkeleton(nullptr), dart::common::NullPointerException);
  EXPECT_EQ(world->getNumSkeletons(), 1u);
}

//==============================================================================
// Test sensor class for sensor management tests
class TestSensor : public sensor::Sensor
{
public:
  using Sensor::Sensor;
  int updateCount = 0;

protected:
  void updateImpl(
      const simulation::World&, const sensor::SensorUpdateContext&) override
  {
    ++updateCount;
  }
};

//==============================================================================
TEST(WorldTests, AddSensor)
{
  auto world = World::create();

  sensor::Sensor::Properties props;
  props.name = "test_sensor";
  auto sensor = std::make_shared<TestSensor>(props);

  EXPECT_EQ(world->getNumSensors(), 0u);

  world->addSensor(sensor);

  EXPECT_EQ(world->getNumSensors(), 1u);
  EXPECT_TRUE(world->hasSensor(sensor));
  EXPECT_TRUE(world->hasSensor("test_sensor"));
}

//==============================================================================
TEST(WorldTests, RemoveSensor)
{
  auto world = World::create();

  sensor::Sensor::Properties props;
  props.name = "removable_sensor";
  auto sensor = std::make_shared<TestSensor>(props);

  world->addSensor(sensor);
  EXPECT_EQ(world->getNumSensors(), 1u);
  EXPECT_TRUE(world->hasSensor(sensor));

  world->removeSensor(sensor);

  EXPECT_EQ(world->getNumSensors(), 0u);
  EXPECT_FALSE(world->hasSensor(sensor));
  EXPECT_FALSE(world->hasSensor("removable_sensor"));
}

//==============================================================================
TEST(WorldTests, GetSensorByIndex)
{
  auto world = World::create();

  sensor::Sensor::Properties props1;
  props1.name = "sensor1";
  auto sensor1 = std::make_shared<TestSensor>(props1);

  sensor::Sensor::Properties props2;
  props2.name = "sensor2";
  auto sensor2 = std::make_shared<TestSensor>(props2);

  world->addSensor(sensor1);
  world->addSensor(sensor2);

  EXPECT_EQ(world->getSensor(0), sensor1);
  EXPECT_EQ(world->getSensor(1), sensor2);
  EXPECT_EQ(world->getSensor(999), nullptr);
}

//==============================================================================
TEST(WorldTests, GetSensorByName)
{
  auto world = World::create();

  sensor::Sensor::Properties props;
  props.name = "named_sensor";
  auto sensor = std::make_shared<TestSensor>(props);

  world->addSensor(sensor);

  EXPECT_EQ(world->getSensor("named_sensor"), sensor);
  EXPECT_EQ(world->getSensor("nonexistent"), nullptr);
}

//==============================================================================
TEST(WorldTests, RemoveAllSensors)
{
  auto world = World::create();

  sensor::Sensor::Properties props1;
  props1.name = "sensor_a";
  auto sensor1 = std::make_shared<TestSensor>(props1);

  sensor::Sensor::Properties props2;
  props2.name = "sensor_b";
  auto sensor2 = std::make_shared<TestSensor>(props2);

  sensor::Sensor::Properties props3;
  props3.name = "sensor_c";
  auto sensor3 = std::make_shared<TestSensor>(props3);

  world->addSensor(sensor1);
  world->addSensor(sensor2);
  world->addSensor(sensor3);

  EXPECT_EQ(world->getNumSensors(), 3u);

  auto removed = world->removeAllSensors();

  EXPECT_EQ(world->getNumSensors(), 0u);
  EXPECT_EQ(removed.size(), 3u);
  EXPECT_TRUE(removed.contains(sensor1));
  EXPECT_TRUE(removed.contains(sensor2));
  EXPECT_TRUE(removed.contains(sensor3));
}

//==============================================================================
TEST(WorldTests, EachSkeleton)
{
  auto world = World::create();

  auto skel1 = createSimpleSkeleton("skel1");
  auto skel2 = createSimpleSkeleton("skel2");
  auto skel3 = createSimpleSkeleton("skel3");

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);
  world->addSkeleton(skel3);

  // Test counting all skeletons
  int counter = 0;
  world->eachSkeleton([&counter](const dynamics::Skeleton*) { ++counter; });
  EXPECT_EQ(counter, 3);

  // Test early termination
  counter = 0;
  world->eachSkeleton([&counter](const dynamics::Skeleton*) -> bool {
    ++counter;
    return counter < 2; // Stop after 2
  });
  EXPECT_EQ(counter, 2);
}

//==============================================================================
TEST(WorldTests, EachSimpleFrame)
{
  auto world = World::create();

  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  auto frame3 = SimpleFrame::createShared(Frame::World(), "frame3");

  world->addSimpleFrame(frame1);
  world->addSimpleFrame(frame2);
  world->addSimpleFrame(frame3);

  // Test counting all frames
  int counter = 0;
  world->eachSimpleFrame(
      [&counter](const dynamics::SimpleFrame*) { ++counter; });
  EXPECT_EQ(counter, 3);

  // Test early termination
  counter = 0;
  world->eachSimpleFrame([&counter](const dynamics::SimpleFrame*) -> bool {
    ++counter;
    return counter < 2; // Stop after 2
  });
  EXPECT_EQ(counter, 2);
}

//==============================================================================
TEST(WorldTests, EachSkeletonEmpty)
{
  auto world = World::create();

  int counter = 0;
  world->eachSkeleton([&counter](const dynamics::Skeleton*) { ++counter; });
  EXPECT_EQ(counter, 0);
}

//==============================================================================
SkeletonPtr createBoxSkeleton(
    const std::string& name, const Eigen::Vector3d& position)
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair.second
      ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  // Set position (FreeJoint has 6 DOFs: 3 rotation + 3 translation)
  Eigen::Vector6d pos = Eigen::Vector6d::Zero();
  pos[3] = position[0];
  pos[4] = position[1];
  pos[5] = position[2];
  skel->setPositions(pos);

  return skel;
}

//==============================================================================
WorldPtr createContactSubstepWorld(
    const std::string& namePrefix,
    double timeStep,
    SkeletonPtr* dynamicBoxOut = nullptr)
{
  auto world = World::create(namePrefix + "_world");
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(timeStep);

  auto ground
      = createBoxSkeleton(namePrefix + "_ground", Eigen::Vector3d(0, 0, -0.5));
  ground->setMobile(false);
  ground->getBodyNode(0)->setMomentOfInertia(0.2, 0.2, 0.2);

  auto box
      = createBoxSkeleton(namePrefix + "_box", Eigen::Vector3d(0, 0, 0.49));
  box->getBodyNode(0)->setMomentOfInertia(0.2, 0.2, 0.2);
  Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
  velocity.head<3>() = Eigen::Vector3d(0.1, -0.2, 0.05);
  velocity.tail<3>() = Eigen::Vector3d(0.05, 0.0, -0.2);
  box->setVelocities(velocity);

  world->addSkeleton(ground);
  world->addSkeleton(box);

  if (dynamicBoxOut) {
    *dynamicBoxOut = box;
  }

  return world;
}

//==============================================================================
WorldPtr createRevoluteChainWorld(
    const std::string& namePrefix,
    double timeStep,
    SkeletonPtr* chainOut = nullptr)
{
  auto world = World::create(namePrefix + "_world");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(timeStep);

  auto chain = Skeleton::create(namePrefix + "_chain");
  chain->disableSelfCollisionCheck();

  constexpr double mass = 1.0;
  constexpr double length = 1.0;
  constexpr double width = 0.05;
  constexpr double transverseMoment
      = mass * (length * length + width * width) / 12.0;
  constexpr double axialMoment = mass * width * width / 6.0;
  BodyNode* parent = nullptr;
  constexpr std::size_t numLinks = 5u;
  for (std::size_t i = 0u; i < numLinks; ++i) {
    RevoluteJoint::Properties jointProperties;
    jointProperties.mName = "joint_" + std::to_string(i);
    jointProperties.mAxis = Eigen::Vector3d::UnitY();
    if (parent) {
      jointProperties.mT_ParentBodyToJoint.translation()
          = Eigen::Vector3d(0.0, 0.0, -length);
    }

    BodyNode::Properties bodyProperties;
    bodyProperties.mName = "link_" + std::to_string(i);
    bodyProperties.mInertia.setMass(mass);
    bodyProperties.mInertia.setLocalCOM(
        Eigen::Vector3d(0.0, 0.0, -0.5 * length));
    bodyProperties.mInertia.setMoment(
        transverseMoment, transverseMoment, axialMoment, 0.0, 0.0, 0.0);

    auto pair = chain->createJointAndBodyNodePair<RevoluteJoint>(
        parent, jointProperties, bodyProperties);
    pair.first->setDampingCoefficient(0, 0.0);
    pair.first->setCoulombFriction(0, 0.0);
    pair.first->setSpringStiffness(0, 0.0);
    pair.first->setLimitEnforcement(false);
    parent = pair.second;
  }

  Eigen::VectorXd positions(chain->getNumDofs());
  Eigen::VectorXd velocities(chain->getNumDofs());
  for (std::size_t i = 0u; i < numLinks; ++i) {
    positions[i] = 0.35 - 0.08 * static_cast<double>(i);
    velocities[i] = 0.4 + 0.05 * static_cast<double>(i);
  }
  chain->setPositions(positions);
  chain->setVelocities(velocities);

  world->addSkeleton(chain);
  if (chainOut) {
    *chainOut = chain;
  }

  return world;
}

//==============================================================================
double totalEnergy(const SkeletonPtr& skeleton)
{
  return skeleton->computeKineticEnergy() + skeleton->computePotentialEnergy();
}

//==============================================================================
double maxRelativeEnergyDrift(
    const WorldPtr& world,
    const SkeletonPtr& skeleton,
    std::size_t numOuterSteps,
    std::size_t substeps)
{
  const double energy0 = totalEnergy(skeleton);
  const double scale = std::max(1.0, std::abs(energy0));
  double maxRelativeDrift = 0.0;

  for (std::size_t i = 0u; i < numOuterSteps; ++i) {
    if (substeps == 1u) {
      world->step();
    } else {
      world->stepSubsteps(substeps);
    }

    const double energy = totalEnergy(skeleton);
    EXPECT_TRUE(std::isfinite(energy));
    maxRelativeDrift
        = std::max(maxRelativeDrift, std::abs(energy - energy0) / scale);
  }

  return maxRelativeDrift;
}

//==============================================================================
double maxRelativeEnergyDriftRungeKutta4(
    const WorldPtr& world, const SkeletonPtr& skeleton, std::size_t numSteps)
{
  const double energy0 = totalEnergy(skeleton);
  const double scale = std::max(1.0, std::abs(energy0));
  double maxRelativeDrift = 0.0;

  for (std::size_t i = 0u; i < numSteps; ++i) {
    world->stepUnconstrainedRungeKutta4();

    const double energy = totalEnergy(skeleton);
    EXPECT_TRUE(std::isfinite(energy));
    maxRelativeDrift
        = std::max(maxRelativeDrift, std::abs(energy - energy0) / scale);
  }

  return maxRelativeDrift;
}

//==============================================================================
double maxPenetrationDepth(const WorldPtr& world)
{
  double maxDepth = 0.0;
  const auto& contacts
      = world->getConstraintSolver()->getLastCollisionResult().getContacts();
  for (const auto& contact : contacts) {
    maxDepth = std::max(maxDepth, contact.penetrationDepth);
  }

  return maxDepth;
}

//==============================================================================
struct ContactStabilitySummary
{
  double maxPenetrationDepth = 0.0;
  double maxHeightError = 0.0;
};

//==============================================================================
ContactStabilitySummary summarizeContactStability(
    const WorldPtr& world,
    const SkeletonPtr& dynamicBox,
    std::size_t numOuterSteps,
    std::size_t substeps)
{
  ContactStabilitySummary summary;
  for (std::size_t i = 0u; i < numOuterSteps; ++i) {
    if (substeps == 1u) {
      world->step();
    } else {
      world->stepSubsteps(substeps);
    }

    summary.maxPenetrationDepth
        = std::max(summary.maxPenetrationDepth, maxPenetrationDepth(world));
    summary.maxHeightError = std::max(
        summary.maxHeightError, std::abs(dynamicBox->getPosition(5) - 0.5));
  }

  return summary;
}

//==============================================================================
constexpr double kContactStabilityBoxSide = 0.4;
constexpr double kContactStabilityBoxHeight = 0.5;
constexpr double kContactStabilityGroundThickness = 0.2;

//==============================================================================
void configureContactStabilityBody(
    BodyNode* body, const std::shared_ptr<BoxShape>& shape, double mass)
{
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  auto* shapeNode
      = body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.0);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.0);
}

//==============================================================================
SkeletonPtr createContactStabilityGround()
{
  auto ground = Skeleton::create("contact_stability_ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<BoxShape>(
      Eigen::Vector3d(6.0, 6.0, kContactStabilityGroundThickness));
  configureContactStabilityBody(body, shape, 1.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation()
      = Eigen::Vector3d(0.0, 0.0, -0.5 * kContactStabilityGroundThickness);
  pair.first->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);

  return ground;
}

//==============================================================================
SkeletonPtr createContactStabilityBox(std::size_t index)
{
  auto skeleton
      = Skeleton::create("contact_stability_box_" + std::to_string(index));
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(
      kContactStabilityBoxSide,
      kContactStabilityBoxSide,
      kContactStabilityBoxHeight));
  configureContactStabilityBody(body, shape, 1.0);

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions[5] = 0.5 * kContactStabilityBoxHeight
                 + static_cast<double>(index) * kContactStabilityBoxHeight;
  skeleton->setPositions(positions);

  return skeleton;
}

//==============================================================================
WorldPtr createContactStabilityWorld(
    double timeStep, std::vector<SkeletonPtr>* boxes)
{
  auto world = World::create();
  world->setTimeStep(timeStep);
  world->addSkeleton(createContactStabilityGround());

  constexpr std::size_t numBoxes = 3u;
  boxes->clear();
  boxes->reserve(numBoxes);
  for (std::size_t i = 0u; i < numBoxes; ++i) {
    auto box = createContactStabilityBox(i);
    world->addSkeleton(box);
    boxes->push_back(std::move(box));
  }

  return world;
}

//==============================================================================
double contactStabilityTopHeightError(const std::vector<SkeletonPtr>& boxes)
{
  const double expected
      = kContactStabilityBoxHeight * (static_cast<double>(boxes.size()) - 0.5);
  const double actual
      = boxes.back()->getBodyNode(0)->getWorldTransform().translation().z();
  return actual - expected;
}

//==============================================================================
ContactStabilitySummary summarizeContactStabilityStack(
    const WorldPtr& world,
    const std::vector<SkeletonPtr>& boxes,
    std::size_t numOuterSteps,
    std::size_t substeps)
{
  ContactStabilitySummary summary;
  for (std::size_t i = 0u; i < numOuterSteps; ++i) {
    if (substeps == 1u) {
      world->step();
    } else {
      world->stepSubsteps(substeps);
    }

    summary.maxPenetrationDepth
        = std::max(summary.maxPenetrationDepth, maxPenetrationDepth(world));
    summary.maxHeightError = std::max(
        summary.maxHeightError,
        std::abs(contactStabilityTopHeightError(boxes)));
  }

  return summary;
}

//==============================================================================
TEST(WorldTests, StepSubstepsMatchesManualSmallerTimeStepWithContact)
{
  constexpr std::size_t substeps = 4u;
  constexpr double outerTimeStep = 0.004;
  constexpr double internalTimeStep
      = outerTimeStep / static_cast<double>(substeps);

  SkeletonPtr substepBox;
  auto substepWorld = createContactSubstepWorld(
      "substep_contact", outerTimeStep, &substepBox);

  SkeletonPtr manualBox;
  auto manualWorld = createContactSubstepWorld(
      "manual_contact", internalTimeStep, &manualBox);

  collision::CollisionOption option(true, 100u);
  EXPECT_TRUE(substepWorld->checkCollision(option));
  EXPECT_TRUE(manualWorld->checkCollision(option));

  substepWorld->stepSubsteps(substeps);
  for (std::size_t i = 0; i < substeps; ++i) {
    manualWorld->step(i + 1u == substeps);
  }

  EXPECT_GT(
      substepWorld->getConstraintSolver()
          ->getLastCollisionResult()
          .getNumContacts(),
      0u);
  EXPECT_EQ(substepWorld->getSimFrames(), manualWorld->getSimFrames());
  EXPECT_NEAR(substepWorld->getTime(), manualWorld->getTime(), 1e-15);
  EXPECT_DOUBLE_EQ(substepWorld->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(substepBox->getTimeStep(), outerTimeStep);
  EXPECT_DOUBLE_EQ(
      substepWorld->getConstraintSolver()->getTimeStep(), outerTimeStep);
  EXPECT_TRUE(
      substepBox->getPositions().isApprox(manualBox->getPositions(), 1e-10))
      << "contact substepping should match manual smaller-timestep positions";
  EXPECT_TRUE(
      substepBox->getVelocities().isApprox(manualBox->getVelocities(), 1e-10))
      << "contact substepping should match manual smaller-timestep velocities";
}

//==============================================================================
TEST(WorldTests, StepSubstepsReducesArticulatedEnergyDrift)
{
  constexpr double outerTimeStep = 0.01;
  constexpr std::size_t numOuterSteps = 500u;
  constexpr std::size_t substeps = 5u;

  SkeletonPtr directChain;
  auto directWorld
      = createRevoluteChainWorld("direct_energy", outerTimeStep, &directChain);

  SkeletonPtr substepChain;
  auto substepWorld = createRevoluteChainWorld(
      "substep_energy", outerTimeStep, &substepChain);

  const double directDrift
      = maxRelativeEnergyDrift(directWorld, directChain, numOuterSteps, 1u);
  const double substepDrift = maxRelativeEnergyDrift(
      substepWorld, substepChain, numOuterSteps, substeps);

  EXPECT_LT(substepDrift, directDrift * 0.5);
  EXPECT_LT(substepDrift, 0.05);
}

//==============================================================================
TEST(WorldTests, StepUnconstrainedRungeKutta4ReducesArticulatedEnergyDrift)
{
  constexpr double timeStep = 0.01;
  constexpr std::size_t numSteps = 500u;
  constexpr std::size_t substeps = 5u;

  SkeletonPtr substepChain;
  auto substepWorld
      = createRevoluteChainWorld("substep_rk4", timeStep, &substepChain);

  SkeletonPtr rungeKuttaChain;
  auto rungeKuttaWorld
      = createRevoluteChainWorld("rk4", timeStep, &rungeKuttaChain);

  const double substepDrift
      = maxRelativeEnergyDrift(substepWorld, substepChain, numSteps, substeps);
  const double rungeKuttaDrift = maxRelativeEnergyDriftRungeKutta4(
      rungeKuttaWorld, rungeKuttaChain, numSteps);

  EXPECT_LT(rungeKuttaDrift, substepDrift * 0.01);
  EXPECT_LT(rungeKuttaDrift, 2e-4);
}

//==============================================================================
TEST(WorldTests, StepUnconstrainedRungeKutta4ResetCommandSemantics)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.004);
  auto [skeleton, body] = createFreeBodySkeleton("rk4_reset_force");
  world->addSkeleton(skeleton);

  auto commandSkeleton = createSimpleSkeleton("rk4_reset_command");
  auto* commandJoint = commandSkeleton->getJoint(0);
  world->addSkeleton(commandSkeleton);

  constexpr double command = 0.5;
  commandJoint->setCommand(0, command);
  body->addExtForce(Eigen::Vector3d(3.0, 0.0, 0.0));
  world->stepUnconstrainedRungeKutta4(false);
  EXPECT_GT(body->getExternalForceLocal().norm(), 0.0);
  EXPECT_DOUBLE_EQ(commandJoint->getCommand(0), command);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.004);
  EXPECT_EQ(world->getSimFrames(), 1);

  world->stepUnconstrainedRungeKutta4(true);
  EXPECT_DOUBLE_EQ(body->getExternalForceLocal().norm(), 0.0);
  EXPECT_DOUBLE_EQ(commandJoint->getCommand(0), 0.0);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.008);
  EXPECT_EQ(world->getSimFrames(), 2);
}

//==============================================================================
TEST(WorldTests, StepSubstepsReducesContactError)
{
  constexpr double outerTimeStep = 0.01;
  constexpr std::size_t numOuterSteps = 1000u;
  constexpr std::size_t substeps = 5u;

  std::vector<SkeletonPtr> directBoxes;
  auto directWorld = createContactStabilityWorld(outerTimeStep, &directBoxes);

  std::vector<SkeletonPtr> substepBoxes;
  auto substepWorld = createContactStabilityWorld(outerTimeStep, &substepBoxes);

  const ContactStabilitySummary directSummary = summarizeContactStabilityStack(
      directWorld, directBoxes, numOuterSteps, 1u);
  const ContactStabilitySummary substepSummary = summarizeContactStabilityStack(
      substepWorld, substepBoxes, numOuterSteps, substeps);

  EXPECT_LT(
      substepSummary.maxPenetrationDepth,
      directSummary.maxPenetrationDepth * 0.5);
  EXPECT_LT(substepSummary.maxHeightError, directSummary.maxHeightError * 0.5);
}

//==============================================================================
TEST(WorldTests, CheckCollisionWithResult)
{
  auto world = World::create();

  // Create two overlapping boxes at the same position
  auto skel1 = createBoxSkeleton("box1", Eigen::Vector3d::Zero());
  auto skel2 = createBoxSkeleton("box2", Eigen::Vector3d(0.5, 0.0, 0.0));

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  collision::CollisionOption option(true, 100u);
  collision::CollisionResult result;

  bool collision = world->checkCollision(option, &result);

  EXPECT_TRUE(collision);
  EXPECT_TRUE(result.isCollision());
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(WorldTests, CheckCollisionNoCollision)
{
  auto world = World::create();

  // Create two separated boxes
  auto skel1 = createBoxSkeleton("box1", Eigen::Vector3d::Zero());
  auto skel2 = createBoxSkeleton("box2", Eigen::Vector3d(10.0, 0.0, 0.0));

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  collision::CollisionOption option(true, 100u);
  collision::CollisionResult result;

  bool collision = world->checkCollision(option, &result);

  EXPECT_FALSE(collision);
  EXPECT_FALSE(result.isCollision());
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(WorldTests, OnNameChangedSignal)
{
  auto world = World::create("original_world");

  std::string capturedOldName;
  std::string capturedNewName;
  bool signalCalled = false;

  world->onNameChanged.connect(
      [&](const std::string& oldName, const std::string& newName) {
        signalCalled = true;
        capturedOldName = oldName;
        capturedNewName = newName;
      });

  world->setName("renamed_world");

  EXPECT_TRUE(signalCalled);
  EXPECT_EQ(capturedOldName, "original_world");
  EXPECT_EQ(capturedNewName, "renamed_world");
}

//==============================================================================
TEST(WorldTests, CloneWithSimpleFrames)
{
  auto world = World::create("frame_world");

  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  frame1->setTranslation(Eigen::Vector3d(1, 2, 3));
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  frame2->setTranslation(Eigen::Vector3d(4, 5, 6));

  world->addSimpleFrame(frame1);
  world->addSimpleFrame(frame2);

  EXPECT_EQ(world->getNumSimpleFrames(), 2u);

  auto clone = world->clone();

  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getNumSimpleFrames(), 2u);
  EXPECT_NE(clone->getSimpleFrame("frame1"), nullptr);
  EXPECT_NE(clone->getSimpleFrame("frame2"), nullptr);
}

//==============================================================================
TEST(WorldTests, SetNameSameNameNoSignal)
{
  auto world = World::create("same_name");

  bool signalCalled = false;
  world->onNameChanged.connect(
      [&](const std::string&, const std::string&) { signalCalled = true; });

  world->setName("same_name");

  EXPECT_FALSE(signalCalled);
  EXPECT_EQ(world->getName(), "same_name");
}

//==============================================================================
TEST(WorldTests, SetTimeStepRejectsInvalid)
{
  auto world = World::create();
  double originalTimeStep = world->getTimeStep();

  world->setTimeStep(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(world->getTimeStep(), originalTimeStep);

  world->setTimeStep(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(world->getTimeStep(), originalTimeStep);

  world->setTimeStep(-std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(world->getTimeStep(), originalTimeStep);
}

//==============================================================================
TEST(WorldTests, GetSimpleFrameOutOfRange)
{
  auto world = World::create();

  auto frame = SimpleFrame::createShared(Frame::World(), "only_frame");
  world->addSimpleFrame(frame);

  EXPECT_EQ(world->getSimpleFrame(0), frame);
  EXPECT_EQ(world->getSimpleFrame(1), nullptr);
  EXPECT_EQ(world->getSimpleFrame(999), nullptr);
}

//==============================================================================
TEST(WorldTests, CreateWithBulletCollisionDetector)
{
  WorldConfig config;
  config.name = "bullet_world";
  config.collisionDetector = CollisionDetectorType::Bullet;
  auto world = World::create(config);
  EXPECT_EQ(world->getName(), "bullet_world");
  EXPECT_NE(world->getCollisionDetector(), nullptr);
}

TEST(WorldTests, AddDuplicateSimpleFrame)
{
  auto world = World::create();
  auto frame = SimpleFrame::createShared(Frame::World(), "dup_frame");

  world->addSimpleFrame(frame);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);

  world->addSimpleFrame(frame);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);
}

TEST(WorldTests, RemoveNonExistentSimpleFrame)
{
  auto world = World::create();
  auto frame1 = SimpleFrame::createShared(Frame::World(), "existing");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "not_added");

  world->addSimpleFrame(frame1);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);

  world->removeSimpleFrame(frame2);
  EXPECT_EQ(world->getNumSimpleFrames(), 1u);
}

TEST(WorldTests, ConstSensorManager)
{
  auto world = World::create();
  const World& constWorld = *world;
  const sensor::SensorManager& mgr = constWorld.getSensorManager();
  (void)mgr;
}

TEST(WorldTests, BakeWithMultipleSkeletons)
{
  auto world = World::create();
  world->setTimeStep(0.001);

  auto skel1 = Skeleton::create("bake_skel1");
  auto pair1 = skel1->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body1"));
  Inertia inertia1;
  inertia1.setMass(1.0);
  pair1.second->setInertia(inertia1);

  auto skel2 = Skeleton::create("bake_skel2");
  auto pair2 = skel2->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("body2"));
  Inertia inertia2;
  inertia2.setMass(1.0);
  pair2.second->setInertia(inertia2);

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  auto recording = world->getRecording();
  EXPECT_EQ(recording->getNumSkeletons(), 2);

  for (int i = 0; i < 5; ++i) {
    world->step();
    world->bake();
  }

  EXPECT_EQ(recording->getNumFrames(), 5);
}

#include <dart/dynamics/weld_joint.hpp>

TEST(WorldTests, DartCollisionDetectorBoxContact)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);

  auto skel1 = createBoxSkeleton("dart_box1", Eigen::Vector3d::Zero());
  auto skel2 = createBoxSkeleton("dart_box2", Eigen::Vector3d::Zero());

  Eigen::Vector6d pos2 = skel2->getPositions();
  pos2[2] = 0.78539816339;
  skel2->setPositions(pos2);

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  collision::CollisionOption option(true, 20u);
  collision::CollisionResult result;
  const bool collided = world->checkCollision(option, &result);

  EXPECT_TRUE(collided);
  EXPECT_TRUE(result.isCollision());
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(WorldTests, FclCollisionDetectorBoxContact)
{
  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Fcl);

  auto skel1 = createBoxSkeleton("fcl_box1", Eigen::Vector3d::Zero());
  auto skel2 = createBoxSkeleton("fcl_box2", Eigen::Vector3d(0.3, 0.0, 0.0));

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);

  collision::CollisionOption option(true, 20u);
  collision::CollisionResult result;
  const bool collided = world->checkCollision(option, &result);

  EXPECT_TRUE(collided);
  EXPECT_TRUE(result.isCollision());
}

TEST(WorldTests, WeldJointWorldStep)
{
  auto world = World::create();
  world->setTimeStep(0.001);

  auto skel = Skeleton::create("weld_world");
  auto rootPair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("root"));
  auto childPair = rootPair.second->createChildJointAndBodyNodePair<WeldJoint>(
      WeldJoint::Properties(), BodyNode::AspectProperties("child"));

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  rootPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);
  childPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

  world->addSkeleton(skel);

  world->step();
  world->step();

  EXPECT_EQ(world->getSimFrames(), 2);
}

TEST(WorldTests, ConstAccessorsAndNameClash)
{
  auto world = World::create("const_world");
  auto skelA = createSimpleSkeleton("robot");
  auto skelB = createSimpleSkeleton("robot");

  world->addSkeleton(skelA);
  world->addSkeleton(skelB);

  EXPECT_NE(skelA->getName(), skelB->getName());

  auto frame = SimpleFrame::createShared(Frame::World(), "const_frame");
  world->addSimpleFrame(frame);

  world->step();

  const World& constWorld = *world;
  EXPECT_EQ(constWorld.getNumSkeletons(), 2u);
  EXPECT_NE(constWorld.getSkeleton(0), nullptr);
  EXPECT_NE(constWorld.getSkeleton(skelA->getName()), nullptr);
  EXPECT_NE(constWorld.getConstraintSolver(), nullptr);
  EXPECT_NE(constWorld.getCollisionDetector(), nullptr);
  EXPECT_NE(constWorld.getSimpleFrame("const_frame"), nullptr);

  EXPECT_EQ(constWorld.getGravity(), world->getGravity());
  EXPECT_DOUBLE_EQ(constWorld.getTimeStep(), world->getTimeStep());
  EXPECT_DOUBLE_EQ(constWorld.getTime(), world->getTime());
  EXPECT_EQ(constWorld.getSimFrames(), world->getSimFrames());

  const auto& lastResult = constWorld.getLastCollisionResult();
  EXPECT_EQ(lastResult.getNumContacts(), 0u);
}
