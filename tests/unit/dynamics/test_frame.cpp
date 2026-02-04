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

//==============================================================================
TEST(SimpleFrameTest, GetLinearVelocity)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d linearVel(1.0, 2.0, 3.0);
  Eigen::Vector3d angularVel(0.1, 0.2, 0.3);

  frame->setClassicDerivatives(linearVel, angularVel);

  Eigen::Vector3d retrievedLinVel = frame->getLinearVelocity();

  EXPECT_FALSE(retrievedLinVel.isZero());
}

//==============================================================================
TEST(SimpleFrameTest, GetAngularVelocity)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d linearVel(1.0, 2.0, 3.0);
  Eigen::Vector3d angularVel(0.5, 0.6, 0.7);

  frame->setClassicDerivatives(linearVel, angularVel);

  Eigen::Vector3d retrievedAngVel = frame->getAngularVelocity();

  EXPECT_FALSE(retrievedAngVel.isZero());
}

//==============================================================================
TEST(SimpleFrameTest, GetLinearAcceleration)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d linearVel(0.0, 0.0, 0.0);
  Eigen::Vector3d angularVel(0.0, 0.0, 0.0);
  Eigen::Vector3d linearAccel(1.0, 2.0, 3.0);
  Eigen::Vector3d angularAccel(0.1, 0.2, 0.3);

  frame->setClassicDerivatives(
      linearVel, angularVel, linearAccel, angularAccel);

  Eigen::Vector3d retrievedLinAccel = frame->getLinearAcceleration();

  EXPECT_FALSE(retrievedLinAccel.isZero());
}

//==============================================================================
TEST(SimpleFrameTest, GetAngularAcceleration)
{
  auto frame = SimpleFrame::createShared();

  Eigen::Vector3d linearVel(0.0, 0.0, 0.0);
  Eigen::Vector3d angularVel(0.0, 0.0, 0.0);
  Eigen::Vector3d linearAccel(0.0, 0.0, 0.0);
  Eigen::Vector3d angularAccel(0.5, 0.6, 0.7);

  frame->setClassicDerivatives(
      linearVel, angularVel, linearAccel, angularAccel);

  Eigen::Vector3d retrievedAngAccel = frame->getAngularAcceleration();

  EXPECT_FALSE(retrievedAngAccel.isZero());
}

//==============================================================================
TEST(SimpleFrameTest, FrameHierarchyParentChild)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");
  auto grandchild = child->spawnChildSimpleFrame("grandchild");

  EXPECT_EQ(parent->getParentFrame(), Frame::World());
  EXPECT_EQ(child->getParentFrame(), parent.get());
  EXPECT_EQ(grandchild->getParentFrame(), child.get());

  EXPECT_EQ(parent->getNumChildFrames(), 1u);
  EXPECT_EQ(child->getNumChildFrames(), 1u);
  EXPECT_EQ(grandchild->getNumChildFrames(), 0u);
}

//==============================================================================
TEST(SimpleFrameTest, DescendsFromAncestry)
{
  auto frameA = SimpleFrame::createShared(Frame::World(), "A");
  auto frameB = frameA->spawnChildSimpleFrame("B");
  auto frameC = frameB->spawnChildSimpleFrame("C");

  EXPECT_TRUE(frameC->descendsFrom(frameB.get()));
  EXPECT_TRUE(frameC->descendsFrom(frameA.get()));
  EXPECT_TRUE(frameC->descendsFrom(Frame::World()));

  EXPECT_TRUE(frameB->descendsFrom(frameA.get()));
  EXPECT_TRUE(frameB->descendsFrom(Frame::World()));
  EXPECT_FALSE(frameB->descendsFrom(frameC.get()));

  EXPECT_TRUE(frameA->descendsFrom(Frame::World()));
  EXPECT_FALSE(frameA->descendsFrom(frameB.get()));
  EXPECT_FALSE(frameA->descendsFrom(frameC.get()));

  EXPECT_TRUE(frameC->descendsFrom(frameC.get()));
  EXPECT_TRUE(frameB->descendsFrom(frameB.get()));
  EXPECT_TRUE(frameA->descendsFrom(frameA.get()));
}

//==============================================================================
TEST(SimpleFrameTest, NestedTransformChain)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = parent->spawnChildSimpleFrame("child");
  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);
  child->setRelativeTransform(childTf);

  auto grandchild = child->spawnChildSimpleFrame("grandchild");
  Eigen::Isometry3d grandchildTf = Eigen::Isometry3d::Identity();
  grandchildTf.translation() = Eigen::Vector3d(0.0, 0.0, 3.0);
  grandchild->setRelativeTransform(grandchildTf);

  Eigen::Isometry3d expectedWorld = parentTf * childTf * grandchildTf;
  EXPECT_TRUE(grandchild->getWorldTransform().isApprox(expectedWorld));

  EXPECT_TRUE(grandchild->getRelativeTransform().isApprox(grandchildTf));
}

//==============================================================================
TEST(SimpleFrameTest, VelocityRelativeToFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d parentLinVel(1.0, 0.0, 0.0);
  Eigen::Vector3d parentAngVel(0.0, 0.0, 0.0);
  parent->setClassicDerivatives(parentLinVel, parentAngVel);

  Eigen::Vector3d childLinVel(0.0, 1.0, 0.0);
  Eigen::Vector3d childAngVel(0.0, 0.0, 0.0);
  child->setClassicDerivatives(childLinVel, childAngVel);

  Eigen::Vector3d childWorldVel = child->getLinearVelocity();
  EXPECT_FALSE(childWorldVel.isZero());

  Eigen::Vector3d childRelVel
      = child->getLinearVelocity(parent.get(), Frame::World());
  EXPECT_FALSE(childRelVel.isZero());
}

//==============================================================================
// Entity Tests - Testing Entity base class functionality via SimpleFrame
//==============================================================================

//==============================================================================
TEST(EntityTest, GetNameAndSetName)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "initial_name");

  EXPECT_EQ(frame->getName(), "initial_name");

  frame->setName("updated_name");
  EXPECT_EQ(frame->getName(), "updated_name");

  // Test empty name
  frame->setName("");
  EXPECT_EQ(frame->getName(), "");

  // Test name with special characters
  frame->setName("frame_with_123_and_special");
  EXPECT_EQ(frame->getName(), "frame_with_123_and_special");
}

//==============================================================================
TEST(EntityTest, GetParentFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  // Test getParentFrame returns correct parent
  EXPECT_EQ(child->getParentFrame(), parent.get());
  EXPECT_EQ(parent->getParentFrame(), Frame::World());

  // Test const version
  const Frame* constParent = child->getParentFrame();
  EXPECT_EQ(constParent, parent.get());
}

//==============================================================================
TEST(EntityTest, SetParentFrameDetachable)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  auto child = frame1->spawnChildSimpleFrame("child");

  EXPECT_EQ(child->getParentFrame(), frame1.get());

  // Change parent frame using setParentFrame (Detachable functionality)
  child->setParentFrame(frame2.get());
  EXPECT_EQ(child->getParentFrame(), frame2.get());

  // Change back to World
  child->setParentFrame(Frame::World());
  EXPECT_EQ(child->getParentFrame(), Frame::World());
}

//==============================================================================
TEST(EntityTest, DescendsFromNullptr)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // descendsFrom(nullptr) should return true per documentation
  EXPECT_TRUE(frame->descendsFrom(nullptr));
}

//==============================================================================
TEST(EntityTest, DescendsFromSelf)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // Entity should descend from itself
  EXPECT_TRUE(frame->descendsFrom(frame.get()));
}

//==============================================================================
TEST(EntityTest, DescendsFromUnrelatedFrame)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");

  // Sibling frames should not descend from each other
  EXPECT_FALSE(frame1->descendsFrom(frame2.get()));
  EXPECT_FALSE(frame2->descendsFrom(frame1.get()));
}

//==============================================================================
TEST(EntityTest, IsFrame)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // SimpleFrame is a Frame, so isFrame() should return true
  EXPECT_TRUE(frame->isFrame());
}

//==============================================================================
TEST(EntityTest, IsQuiet)
{
  // Default SimpleFrame is not quiet
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");
  EXPECT_FALSE(frame->isQuiet());
}

//==============================================================================
TEST(EntityTest, NeedsTransformUpdate)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // After creation, transform update may be needed
  // After getting transform, it should be updated
  [[maybe_unused]] auto tf = frame->getWorldTransform();
  EXPECT_FALSE(frame->needsTransformUpdate());

  // After dirtying, update should be needed
  frame->dirtyTransform();
  EXPECT_TRUE(frame->needsTransformUpdate());
}

//==============================================================================
TEST(EntityTest, NeedsVelocityUpdate)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // After getting velocity, it should be updated
  [[maybe_unused]] auto vel = frame->getSpatialVelocity();
  EXPECT_FALSE(frame->needsVelocityUpdate());

  // After dirtying, update should be needed
  frame->dirtyVelocity();
  EXPECT_TRUE(frame->needsVelocityUpdate());
}

//==============================================================================
TEST(EntityTest, NeedsAccelerationUpdate)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // After getting acceleration, it should be updated
  [[maybe_unused]] auto accel = frame->getSpatialAcceleration();
  EXPECT_FALSE(frame->needsAccelerationUpdate());

  // After dirtying, update should be needed
  frame->dirtyAcceleration();
  EXPECT_TRUE(frame->needsAccelerationUpdate());
}

//==============================================================================
TEST(EntityTest, FrameChangedSignal)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  auto child = frame1->spawnChildSimpleFrame("child");

  bool signalReceived = false;
  const Frame* oldFrameReceived = nullptr;
  const Frame* newFrameReceived = nullptr;

  // Connect to the frame changed signal
  auto connection = child->onFrameChanged.connect(
      [&](const Entity*, const Frame* oldFrame, const Frame* newFrame) {
        signalReceived = true;
        oldFrameReceived = oldFrame;
        newFrameReceived = newFrame;
      });

  // Change parent frame
  child->setParentFrame(frame2.get());

  EXPECT_TRUE(signalReceived);
  EXPECT_EQ(oldFrameReceived, frame1.get());
  EXPECT_EQ(newFrameReceived, frame2.get());
}

//==============================================================================
TEST(EntityTest, NameChangedSignal)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "original");

  bool signalReceived = false;
  std::string oldNameReceived;
  std::string newNameReceived;

  // Connect to the name changed signal
  auto connection
      = frame->onNameChanged.connect([&](const Entity*,
                                         const std::string& oldName,
                                         const std::string& newName) {
          signalReceived = true;
          oldNameReceived = oldName;
          newNameReceived = newName;
        });

  // Change name
  frame->setName("renamed");

  EXPECT_TRUE(signalReceived);
  EXPECT_EQ(oldNameReceived, "original");
  EXPECT_EQ(newNameReceived, "renamed");
}

//==============================================================================
TEST(EntityTest, TransformUpdatedSignal)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  bool signalReceived = false;

  // Connect to the transform updated signal
  auto connection = frame->onTransformUpdated.connect(
      [&](const Entity*) { signalReceived = true; });

  // Dirty the transform (should trigger signal)
  frame->dirtyTransform();

  EXPECT_TRUE(signalReceived);
}

//==============================================================================
TEST(EntityTest, VelocityChangedSignal)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  bool signalReceived = false;

  // Connect to the velocity changed signal
  auto connection = frame->onVelocityChanged.connect(
      [&](const Entity*) { signalReceived = true; });

  // Dirty the velocity (should trigger signal)
  frame->dirtyVelocity();

  EXPECT_TRUE(signalReceived);
}

//==============================================================================
TEST(EntityTest, AccelerationChangedSignal)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  bool signalReceived = false;

  // Connect to the acceleration changed signal
  auto connection = frame->onAccelerationChanged.connect(
      [&](const Entity*) { signalReceived = true; });

  // Dirty the acceleration (should trigger signal)
  frame->dirtyAcceleration();

  EXPECT_TRUE(signalReceived);
}

//==============================================================================
TEST(EntityTest, ParentFrameChangePropagatesToChildren)
{
  auto grandparent = SimpleFrame::createShared(Frame::World(), "grandparent");
  auto parent = grandparent->spawnChildSimpleFrame("parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Isometry3d gpTf = Eigen::Isometry3d::Identity();
  gpTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  grandparent->setRelativeTransform(gpTf);

  Eigen::Isometry3d pTf = Eigen::Isometry3d::Identity();
  pTf.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  parent->setRelativeTransform(pTf);

  Eigen::Isometry3d cTf = Eigen::Isometry3d::Identity();
  cTf.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  child->setRelativeTransform(cTf);

  Eigen::Isometry3d expectedWorld = gpTf * pTf * cTf;
  EXPECT_TRUE(child->getWorldTransform().isApprox(expectedWorld));

  EXPECT_TRUE(child->descendsFrom(parent.get()));
  EXPECT_TRUE(child->descendsFrom(grandparent.get()));
  EXPECT_TRUE(parent->descendsFrom(grandparent.get()));
}

//==============================================================================
TEST(FrameTest, GetTransformThreeArgs)
{
  auto frameA = SimpleFrame::createShared(Frame::World(), "A");
  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  tfA.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  frameA->setRelativeTransform(tfA);

  auto frameB = SimpleFrame::createShared(Frame::World(), "B");
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);
  frameB->setRelativeTransform(tfB);

  auto frameC = SimpleFrame::createShared(Frame::World(), "C");
  Eigen::Isometry3d tfC = Eigen::Isometry3d::Identity();
  tfC.translation() = Eigen::Vector3d(0.0, 0.0, 3.0);
  frameC->setRelativeTransform(tfC);

  Eigen::Isometry3d result = frameA->getTransform(frameB.get(), frameC.get());
  EXPECT_TRUE(result.matrix().allFinite());

  Eigen::Isometry3d sameFrame
      = frameA->getTransform(frameB.get(), frameB.get());
  Eigen::Isometry3d simple = frameA->getTransform(frameB.get());
  EXPECT_TRUE(sameFrame.isApprox(simple, 1e-10));
}

//==============================================================================
TEST(FrameTest, GetTransformSelfReference)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "self");
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(5.0, 6.0, 7.0);
  frame->setRelativeTransform(tf);

  Eigen::Isometry3d selfTf = frame->getTransform(frame.get());
  EXPECT_TRUE(selfTf.isApprox(Eigen::Isometry3d::Identity(), 1e-10));
}

//==============================================================================
TEST(FrameTest, GetTransformParentReference)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = parent->spawnChildSimpleFrame("child");
  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);
  child->setRelativeTransform(childTf);

  Eigen::Isometry3d relTf = child->getTransform(parent.get());
  EXPECT_TRUE(relTf.isApprox(childTf, 1e-10));
}

//==============================================================================
TEST(FrameTest, SpatialVelocityWithOffset)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "vel_offset");

  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linVel(0.0, 0.0, 0.0);
  frame->setClassicDerivatives(linVel, angVel);

  Eigen::Vector3d offset(1.0, 0.0, 0.0);
  Eigen::Vector6d velWithOffset = frame->getSpatialVelocity(offset);
  EXPECT_TRUE(velWithOffset.allFinite());
  EXPECT_FALSE(velWithOffset.isZero());
}

//==============================================================================
TEST(FrameTest, SpatialVelocityWithOffsetRelativeTo)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linVel(1.0, 0.0, 0.0);
  child->setClassicDerivatives(linVel, angVel);

  Eigen::Vector3d offset(0.5, 0.0, 0.0);

  Eigen::Vector6d velWorld
      = child->getSpatialVelocity(offset, Frame::World(), child.get());
  EXPECT_TRUE(velWorld.allFinite());

  Eigen::Vector6d velRelParent
      = child->getSpatialVelocity(offset, parent.get(), child.get());
  EXPECT_TRUE(velRelParent.allFinite());

  Eigen::Vector6d velRelSelf
      = child->getSpatialVelocity(offset, child.get(), child.get());
  EXPECT_TRUE(velRelSelf.isZero());
}

//==============================================================================
TEST(FrameTest, SpatialVelocityRelativeToNonWorld)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linVel(1.0, 0.0, 0.0);
  parent->setClassicDerivatives(linVel, angVel);

  Eigen::Vector3d childLinVel(0.0, 1.0, 0.0);
  Eigen::Vector3d childAngVel(0.0, 0.0, 0.5);
  child->setClassicDerivatives(childLinVel, childAngVel);

  Eigen::Vector6d velRelParent
      = child->getSpatialVelocity(parent.get(), child.get());
  EXPECT_TRUE(velRelParent.allFinite());

  Eigen::Vector6d velRelParentInWorld
      = child->getSpatialVelocity(parent.get(), Frame::World());
  EXPECT_TRUE(velRelParentInWorld.allFinite());
}

//==============================================================================
TEST(FrameTest, SpatialAccelerationWithOffset)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "acc_offset");

  Eigen::Vector3d linVel(0.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linAcc(1.0, 0.0, 0.0);
  Eigen::Vector3d angAcc(0.0, 0.0, 0.5);
  frame->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector3d offset(1.0, 0.0, 0.0);
  Eigen::Vector6d accWithOffset = frame->getSpatialAcceleration(offset);
  EXPECT_TRUE(accWithOffset.allFinite());
}

//==============================================================================
TEST(FrameTest, SpatialAccelerationWithOffsetRelativeTo)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d linVel(0.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linAcc(1.0, 0.0, 0.0);
  Eigen::Vector3d angAcc(0.0, 0.0, 0.5);
  child->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector3d offset(0.5, 0.0, 0.0);

  Eigen::Vector6d accWorld
      = child->getSpatialAcceleration(offset, Frame::World(), child.get());
  EXPECT_TRUE(accWorld.allFinite());

  Eigen::Vector6d accRelParent
      = child->getSpatialAcceleration(offset, parent.get(), child.get());
  EXPECT_TRUE(accRelParent.allFinite());

  Eigen::Vector6d accRelSelf
      = child->getSpatialAcceleration(offset, child.get(), child.get());
  EXPECT_TRUE(accRelSelf.isZero());
}

//==============================================================================
TEST(FrameTest, SpatialAccelerationRelativeToNonWorld)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d linVel(1.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linAcc(0.5, 0.0, 0.0);
  Eigen::Vector3d angAcc(0.0, 0.0, 0.2);
  parent->setClassicDerivatives(linVel, angVel, linAcc, angAcc);
  child->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector6d accRelParent
      = child->getSpatialAcceleration(parent.get(), child.get());
  EXPECT_TRUE(accRelParent.allFinite());

  Eigen::Vector6d accRelParentInWorld
      = child->getSpatialAcceleration(parent.get(), Frame::World());
  EXPECT_TRUE(accRelParentInWorld.allFinite());
}

//==============================================================================
TEST(FrameTest, LinearAccelerationWithOffset)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d linVel(1.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linAcc(0.5, 0.0, 0.0);
  Eigen::Vector3d angAcc(0.0, 0.0, 0.2);
  child->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector3d offset(0.5, 0.0, 0.0);

  Eigen::Vector3d linAccWorld
      = child->getLinearAcceleration(offset, Frame::World(), child.get());
  EXPECT_TRUE(linAccWorld.allFinite());

  Eigen::Vector3d linAccRelParent
      = child->getLinearAcceleration(offset, parent.get(), child.get());
  EXPECT_TRUE(linAccRelParent.allFinite());

  Eigen::Vector3d linAccRelSelf
      = child->getLinearAcceleration(offset, child.get(), child.get());
  EXPECT_TRUE(linAccRelSelf.isZero());
}

//==============================================================================
TEST(FrameTest, LinearAccelerationInDifferentCoordinates)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  Eigen::Vector3d linVel(1.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 0.5);
  Eigen::Vector3d linAcc(0.5, 0.0, 0.0);
  Eigen::Vector3d angAcc(0.0, 0.0, 0.1);
  child->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector3d linAccInWorld
      = child->getLinearAcceleration(Frame::World(), Frame::World());
  EXPECT_TRUE(linAccInWorld.allFinite());

  Eigen::Vector3d linAccInChild
      = child->getLinearAcceleration(Frame::World(), child.get());
  EXPECT_TRUE(linAccInChild.allFinite());

  Eigen::Vector3d linAccRelParent
      = child->getLinearAcceleration(parent.get(), Frame::World());
  EXPECT_TRUE(linAccRelParent.allFinite());

  Eigen::Vector3d linAccRelSelf
      = child->getLinearAcceleration(child.get(), child.get());
  EXPECT_TRUE(linAccRelSelf.isZero());
}

//==============================================================================
TEST(FrameTest, GetLinearVelocityWithOffset)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "lin_vel_offset");

  Eigen::Vector3d angVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linVel(0.0, 0.0, 0.0);
  frame->setClassicDerivatives(linVel, angVel);

  Eigen::Vector3d offset(1.0, 0.0, 0.0);
  Eigen::Vector3d linVelWithOffset
      = frame->getLinearVelocity(offset, Frame::World(), frame.get());
  EXPECT_TRUE(linVelWithOffset.allFinite());
}

//==============================================================================
TEST(FrameTest, WorldFrameSetNameRejected)
{
  Frame* world = Frame::World();
  const std::string& name = world->setName("NotWorld");
  EXPECT_EQ(name, "World");
  EXPECT_EQ(world->getName(), "World");
}

//==============================================================================
TEST(FrameTest, WorldFramePartialAcceleration)
{
  Frame* world = Frame::World();
  const Eigen::Vector6d& partialAcc = world->getPartialAcceleration();
  EXPECT_TRUE(partialAcc.isZero());
}

//==============================================================================
TEST(FrameTest, WorldFramePrimaryRelativeAcceleration)
{
  Frame* world = Frame::World();
  const Eigen::Vector6d& primAcc = world->getPrimaryRelativeAcceleration();
  EXPECT_TRUE(primAcc.isZero());
}

//==============================================================================
TEST(FrameTest, ChildFramesSets)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child1 = parent->spawnChildSimpleFrame("child1");
  auto child2 = parent->spawnChildSimpleFrame("child2");

  const std::set<Frame*>& childFrames = parent->getChildFrames();
  EXPECT_EQ(childFrames.size(), 2u);
  EXPECT_TRUE(childFrames.count(child1.get()) > 0);
  EXPECT_TRUE(childFrames.count(child2.get()) > 0);

  const std::set<const Frame*> constChildFrames
      = static_cast<const Frame*>(parent.get())->getChildFrames();
  EXPECT_EQ(constChildFrames.size(), 2u);
}

//==============================================================================
TEST(FrameTest, ChildEntitiesSets)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  const std::set<Entity*>& childEntities = parent->getChildEntities();
  EXPECT_GE(childEntities.size(), 1u);

  const std::set<const Entity*> constChildEntities
      = static_cast<const Frame*>(parent.get())->getChildEntities();
  EXPECT_GE(constChildEntities.size(), 1u);
}

//==============================================================================
TEST(FrameTest, DirtyTransformPropagation)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  [[maybe_unused]] auto tf = child->getWorldTransform();
  EXPECT_FALSE(child->needsTransformUpdate());

  parent->dirtyTransform();
  EXPECT_TRUE(child->needsTransformUpdate());
}

//==============================================================================
TEST(FrameTest, DirtyVelocityPropagation)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  [[maybe_unused]] auto vel = child->getSpatialVelocity();
  EXPECT_FALSE(child->needsVelocityUpdate());

  parent->dirtyVelocity();
  EXPECT_TRUE(child->needsVelocityUpdate());
}

//==============================================================================
TEST(FrameTest, DirtyAccelerationPropagation)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = parent->spawnChildSimpleFrame("child");

  [[maybe_unused]] auto acc = child->getSpatialAcceleration();
  EXPECT_FALSE(child->needsAccelerationUpdate());

  parent->dirtyAcceleration();
  EXPECT_TRUE(child->needsAccelerationUpdate());
}

//==============================================================================
TEST(FrameTest, DirtyTransformIdempotent)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  frame->dirtyTransform();
  EXPECT_TRUE(frame->needsTransformUpdate());

  frame->dirtyTransform();
  EXPECT_TRUE(frame->needsTransformUpdate());
}

//==============================================================================
TEST(FrameTest, DirtyVelocityIdempotent)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  frame->dirtyVelocity();
  EXPECT_TRUE(frame->needsVelocityUpdate());

  frame->dirtyVelocity();
  EXPECT_TRUE(frame->needsVelocityUpdate());
}

//==============================================================================
TEST(FrameTest, DirtyAccelerationIdempotent)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  frame->dirtyAcceleration();
  EXPECT_TRUE(frame->needsAccelerationUpdate());

  frame->dirtyAcceleration();
  EXPECT_TRUE(frame->needsAccelerationUpdate());
}

//==============================================================================
TEST(FrameTest, SpatialVelocityRelativeToWorldInWorldCoords)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector3d linVel(1.0, 2.0, 3.0);
  Eigen::Vector3d angVel(0.1, 0.2, 0.3);
  frame->setClassicDerivatives(linVel, angVel);

  Eigen::Vector6d velInWorld
      = frame->getSpatialVelocity(Frame::World(), Frame::World());
  EXPECT_TRUE(velInWorld.allFinite());
}

//==============================================================================
TEST(FrameTest, SpatialAccelerationRelativeToWorldInWorldCoords)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector3d linVel(0.0, 0.0, 0.0);
  Eigen::Vector3d angVel(0.0, 0.0, 0.0);
  Eigen::Vector3d linAcc(1.0, 2.0, 3.0);
  Eigen::Vector3d angAcc(0.1, 0.2, 0.3);
  frame->setClassicDerivatives(linVel, angVel, linAcc, angAcc);

  Eigen::Vector6d accInWorld
      = frame->getSpatialAcceleration(Frame::World(), Frame::World());
  EXPECT_TRUE(accInWorld.allFinite());
}
