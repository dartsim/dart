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

#include <dart/simulation/recording.hpp>
#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/common/exception.hpp>

#include <dart/sensor/sensor.hpp>

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
  EXPECT_TRUE(removed.count(sensor1) > 0);
  EXPECT_TRUE(removed.count(sensor2) > 0);
  EXPECT_TRUE(removed.count(sensor3) > 0);
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
  pair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

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
