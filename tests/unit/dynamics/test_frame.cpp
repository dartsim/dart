/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
TEST(FrameTest, WorldFrameSingleton)
{
  Frame* world1 = Frame::World();
  Frame* world2 = Frame::World();

  EXPECT_EQ(world1, world2);
  EXPECT_TRUE(world1->isWorld());
  EXPECT_EQ(world1->getName(), "World");
}

//==============================================================================
TEST(FrameTest, WorldFrameSharedPointer)
{
  auto worldShared1 = Frame::WorldShared();
  auto worldShared2 = Frame::WorldShared();

  ASSERT_NE(worldShared1, nullptr);
  ASSERT_NE(worldShared2, nullptr);
  EXPECT_TRUE(worldShared1->isWorld());
  EXPECT_TRUE(worldShared2->isWorld());
}

//==============================================================================
TEST(FrameTest, WorldFrameTransformIsIdentity)
{
  Frame* world = Frame::World();

  const Eigen::Isometry3d& transform = world->getWorldTransform();
  EXPECT_TRUE(transform.isApprox(Eigen::Isometry3d::Identity()));

  const Eigen::Isometry3d& relTransform = world->getRelativeTransform();
  EXPECT_TRUE(relTransform.isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
TEST(FrameTest, WorldFrameVelocityIsZero)
{
  Frame* world = Frame::World();

  const Eigen::Vector6d& velocity = world->getSpatialVelocity();
  EXPECT_TRUE(velocity.isZero());

  const Eigen::Vector6d& relVelocity = world->getRelativeSpatialVelocity();
  EXPECT_TRUE(relVelocity.isZero());
}

//==============================================================================
TEST(FrameTest, WorldFrameAccelerationIsZero)
{
  Frame* world = Frame::World();

  const Eigen::Vector6d& accel = world->getSpatialAcceleration();
  EXPECT_TRUE(accel.isZero());

  const Eigen::Vector6d& relAccel = world->getRelativeSpatialAcceleration();
  EXPECT_TRUE(relAccel.isZero());
}

//==============================================================================
TEST(FrameTest, WorldFrameIsItsOwnParent)
{
  Frame* world = Frame::World();

  EXPECT_EQ(world->getParentFrame(), world);
}

//==============================================================================
TEST(SimpleFrameTest, DefaultConstruction)
{
  auto frame = SimpleFrame::createShared();

  EXPECT_EQ(frame->getParentFrame(), Frame::World());
  EXPECT_EQ(frame->getName(), "simple_frame");
  EXPECT_TRUE(
      frame->getRelativeTransform().isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
TEST(SimpleFrameTest, ConstructionWithName)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "my_frame");

  EXPECT_EQ(frame->getName(), "my_frame");
}

//==============================================================================
TEST(SimpleFrameTest, ConstructionWithTransform)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto frame = SimpleFrame::createShared(Frame::World(), "test", transform);

  EXPECT_TRUE(frame->getRelativeTransform().isApprox(transform));
}

//==============================================================================
TEST(SimpleFrameTest, SetName)
{
  auto frame = SimpleFrame::createShared();

  frame->setName("new_name");
  EXPECT_EQ(frame->getName(), "new_name");
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeTransform)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(5.0, 6.0, 7.0);

  frame->setRelativeTransform(transform);

  EXPECT_TRUE(frame->getRelativeTransform().isApprox(transform));
  EXPECT_TRUE(frame->getWorldTransform().isApprox(transform));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeTranslation)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d translation(1.0, 2.0, 3.0);
  frame->setRelativeTranslation(translation);

  EXPECT_TRUE(
      frame->getRelativeTransform().translation().isApprox(translation));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeRotation)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(math::pi / 4.0, Eigen::Vector3d::UnitZ());

  frame->setRelativeRotation(rotation);

  EXPECT_TRUE(frame->getRelativeTransform().rotation().isApprox(rotation));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialVelocity)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector6d velocity;
  velocity << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  frame->setRelativeSpatialVelocity(velocity);

  EXPECT_TRUE(frame->getRelativeSpatialVelocity().isApprox(velocity));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialAcceleration)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector6d accel;
  accel << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  frame->setRelativeSpatialAcceleration(accel);

  EXPECT_TRUE(frame->getRelativeSpatialAcceleration().isApprox(accel));
}

//==============================================================================
TEST(SimpleFrameTest, Clone)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto frame = SimpleFrame::createShared(Frame::World(), "original", transform);

  auto cloned = frame->clone();

  EXPECT_NE(cloned.get(), frame.get());
  EXPECT_TRUE(cloned->getWorldTransform().isApprox(frame->getWorldTransform()));
}

//==============================================================================
TEST(SimpleFrameTest, Copy)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto source = SimpleFrame::createShared(Frame::World(), "source", transform);
  auto dest = SimpleFrame::createShared();

  dest->copy(*source);

  EXPECT_TRUE(dest->getWorldTransform().isApprox(source->getWorldTransform()));
}

//==============================================================================
TEST(SimpleFrameTest, SpawnChildFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");

  auto child = parent->spawnChildSimpleFrame("child");

  EXPECT_EQ(child->getParentFrame(), parent.get());
  EXPECT_EQ(child->getName(), "child");
}

//==============================================================================
TEST(SimpleFrameTest, NestedFrameTransforms)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = parent->spawnChildSimpleFrame("child");
  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  child->setRelativeTransform(childTf);

  Eigen::Isometry3d expectedWorld = parentTf * childTf;
  EXPECT_TRUE(child->getWorldTransform().isApprox(expectedWorld));
}

//==============================================================================
TEST(SimpleFrameTest, GetTransformWithRespectTo)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  frame1->setRelativeTransform(tf1);

  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  frame2->setRelativeTransform(tf2);

  Eigen::Isometry3d relativeTransform = frame2->getTransform(frame1.get());
  Eigen::Vector3d expectedTranslation(1.0, 0.0, 0.0);

  EXPECT_TRUE(relativeTransform.translation().isApprox(expectedTranslation));
}

//==============================================================================
TEST(SimpleFrameTest, IsShapeFrame)
{
  auto frame = SimpleFrame::createShared();

  EXPECT_TRUE(frame->isShapeFrame());
  EXPECT_NE(frame->asShapeFrame(), nullptr);
}

//==============================================================================
TEST(SimpleFrameTest, IsNotWorld)
{
  auto frame = SimpleFrame::createShared();

  EXPECT_FALSE(frame->isWorld());
}

//==============================================================================
TEST(SimpleFrameTest, ChildEntityTracking)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");

  EXPECT_EQ(parent->getNumChildFrames(), 0u);
  EXPECT_EQ(parent->getNumChildEntities(), 0u);

  auto child = parent->spawnChildSimpleFrame("child");

  EXPECT_EQ(parent->getNumChildFrames(), 1u);
  EXPECT_GE(parent->getNumChildEntities(), 1u);
}

//==============================================================================
TEST(SimpleFrameTest, SetClassicDerivatives)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d linearVel(1.0, 2.0, 3.0);
  Eigen::Vector3d angularVel(0.1, 0.2, 0.3);
  Eigen::Vector3d linearAccel(0.01, 0.02, 0.03);
  Eigen::Vector3d angularAccel(0.001, 0.002, 0.003);

  frame->setClassicDerivatives(
      linearVel, angularVel, linearAccel, angularAccel);

  EXPECT_FALSE(frame->getRelativeSpatialVelocity().isZero());
  EXPECT_FALSE(frame->getRelativeSpatialAcceleration().isZero());
}

//==============================================================================
TEST(SimpleFrameTest, SetTransformWithRespectTo)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Isometry3d worldTf = Eigen::Isometry3d::Identity();
  worldTf.translation() = Eigen::Vector3d(15.0, 0.0, 0.0);

  child->setTransform(worldTf, Frame::World());

  EXPECT_TRUE(child->getWorldTransform().isApprox(worldTf, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, PartialAccelerationIsZero)
{
  auto frame = SimpleFrame::createShared();

  EXPECT_TRUE(frame->getPartialAcceleration().isZero());
}

//==============================================================================
TEST(SimpleFrameTest, PrimaryRelativeAcceleration)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector6d accel;
  accel << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  frame->setRelativeSpatialAcceleration(accel);

  EXPECT_TRUE(frame->getPrimaryRelativeAcceleration().isApprox(accel));
}
