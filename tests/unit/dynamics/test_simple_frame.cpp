// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/simple_frame.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
TEST(SimpleFrameTest, Constructor)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "test_frame");

  EXPECT_EQ(frame->getName(), "test_frame");
  EXPECT_EQ(frame->getParentFrame(), Frame::World());
  EXPECT_TRUE(
      frame->getRelativeTransform().isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
TEST(SimpleFrameTest, ConstructorWithTransform)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto frame
      = SimpleFrame::createShared(Frame::World(), "translated_frame", tf);

  EXPECT_TRUE(frame->getRelativeTransform().isApprox(tf));
  EXPECT_TRUE(frame->getTransform().translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(SimpleFrameTest, SetAndGetName)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "initial_name");

  frame->setName("new_name");

  EXPECT_EQ(frame->getName(), "new_name");
}

//==============================================================================
TEST(SimpleFrameTest, SetNameReturnsSameName)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "test_frame");

  const std::string& result = frame->setName("same_name");
  EXPECT_EQ(result, "same_name");

  const std::string& sameResult = frame->setName("same_name");
  EXPECT_EQ(sameResult, "same_name");
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeTransform)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Isometry3d newTf = Eigen::Isometry3d::Identity();
  newTf.translation() = Eigen::Vector3d(5.0, 6.0, 7.0);
  newTf.linear()
      = Eigen::AngleAxisd(dart::math::pi / 4, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

  frame->setRelativeTransform(newTf);

  EXPECT_TRUE(frame->getRelativeTransform().isApprox(newTf));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeTranslation)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector3d translation(10.0, 20.0, 30.0);
  frame->setRelativeTranslation(translation);

  EXPECT_TRUE(
      frame->getRelativeTransform().translation().isApprox(translation));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeRotation)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Matrix3d rotation
      = Eigen::AngleAxisd(dart::math::pi / 2, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
  frame->setRelativeRotation(rotation);

  EXPECT_TRUE(frame->getRelativeTransform().linear().isApprox(rotation));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialVelocity)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector6d velocity;
  velocity << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;

  frame->setRelativeSpatialVelocity(velocity);

  EXPECT_TRUE(frame->getRelativeSpatialVelocity().isApprox(velocity));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialAcceleration)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector6d acceleration;
  acceleration << 0.01, 0.02, 0.03, 0.1, 0.2, 0.3;

  frame->setRelativeSpatialAcceleration(acceleration);

  EXPECT_TRUE(frame->getRelativeSpatialAcceleration().isApprox(acceleration));
  EXPECT_TRUE(frame->getPrimaryRelativeAcceleration().isApprox(acceleration));
}

//==============================================================================
TEST(SimpleFrameTest, Clone)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto original = SimpleFrame::createShared(Frame::World(), "original", tf);
  auto cloned = original->clone(Frame::World());

  EXPECT_NE(cloned.get(), original.get());
  EXPECT_TRUE(cloned->getRelativeTransform().isApprox(tf));
}

//==============================================================================
TEST(SimpleFrameTest, SpawnChildSimpleFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");

  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  auto child = parent->spawnChildSimpleFrame("child", childTf);

  EXPECT_EQ(child->getName(), "child");
  EXPECT_EQ(child->getParentFrame(), parent.get());
  EXPECT_TRUE(child->getRelativeTransform().isApprox(childTf));
}

//==============================================================================
TEST(SimpleFrameTest, CopyFrame)
{
  auto source = SimpleFrame::createShared(Frame::World(), "source");
  Eigen::Isometry3d sourceTf = Eigen::Isometry3d::Identity();
  sourceTf.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  source->setRelativeTransform(sourceTf);

  auto dest = SimpleFrame::createShared(Frame::World(), "dest");
  dest->copy(*source, Frame::World());

  EXPECT_TRUE(dest->getTransform().isApprox(source->getTransform(), 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, AssignmentOperator)
{
  auto source = SimpleFrame::createShared(Frame::World(), "source");
  Eigen::Isometry3d sourceTf = Eigen::Isometry3d::Identity();
  sourceTf.translation() = Eigen::Vector3d(3.0, 4.0, 5.0);
  source->setRelativeTransform(sourceTf);

  auto dest = SimpleFrame::createShared(Frame::World(), "dest");
  *dest = *source;

  EXPECT_TRUE(dest->getTransform().isApprox(source->getTransform(), 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, SetTransformWithRespectTo)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");

  Eigen::Isometry3d worldTf = Eigen::Isometry3d::Identity();
  worldTf.translation() = Eigen::Vector3d(15.0, 0.0, 0.0);
  child->setTransform(worldTf, Frame::World());

  EXPECT_TRUE(child->getTransform(Frame::World()).isApprox(worldTf, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, SetClassicDerivatives)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector3d linearVel(1.0, 2.0, 3.0);
  Eigen::Vector3d angularVel(0.1, 0.2, 0.3);
  Eigen::Vector3d linearAcc(0.5, 0.5, 0.5);
  Eigen::Vector3d angularAcc(0.01, 0.02, 0.03);

  frame->setClassicDerivatives(linearVel, angularVel, linearAcc, angularAcc);

  Eigen::Vector6d expectedVel;
  expectedVel << angularVel, linearVel;
  EXPECT_TRUE(frame->getRelativeSpatialVelocity().isApprox(expectedVel, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialVelocityInCoordinatesOf)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector6d velocity;
  velocity << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;

  frame->setRelativeSpatialVelocity(velocity, frame.get());

  EXPECT_TRUE(frame->getRelativeSpatialVelocity().isApprox(velocity));
}

//==============================================================================
TEST(SimpleFrameTest, SetParentFrame)
{
  auto parent1 = SimpleFrame::createShared(Frame::World(), "parent1");
  auto parent2 = SimpleFrame::createShared(Frame::World(), "parent2");
  auto child = SimpleFrame::createShared(parent1.get(), "child");

  EXPECT_EQ(child->getParentFrame(), parent1.get());

  child->setParentFrame(parent2.get());

  EXPECT_EQ(child->getParentFrame(), parent2.get());
}

//==============================================================================
TEST(SimpleFrameTest, SetParentFrameToWorld)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  auto child = SimpleFrame::createShared(parent.get(), "child");

  EXPECT_EQ(child->getParentFrame(), parent.get());

  child->setParentFrame(Frame::World());

  EXPECT_EQ(child->getParentFrame(), Frame::World());
}

//==============================================================================
TEST(SimpleFrameTest, CloneWithDifferentReferenceFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto original = SimpleFrame::createShared(Frame::World(), "original");
  Eigen::Isometry3d originalTf = Eigen::Isometry3d::Identity();
  originalTf.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  original->setRelativeTransform(originalTf);

  // Clone with parent as reference frame
  auto cloned = original->clone(parent.get());

  EXPECT_NE(cloned.get(), original.get());
  EXPECT_EQ(cloned->getParentFrame(), parent.get());
  // World transform should be preserved
  EXPECT_TRUE(
      cloned->getWorldTransform().isApprox(original->getWorldTransform()));
}

//==============================================================================
TEST(SimpleFrameTest, ClonePreservesVelocityAndAcceleration)
{
  auto original = SimpleFrame::createShared(Frame::World(), "original");

  Eigen::Vector6d velocity;
  velocity << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;
  original->setRelativeSpatialVelocity(velocity);

  Eigen::Vector6d acceleration;
  acceleration << 0.01, 0.02, 0.03, 0.1, 0.2, 0.3;
  original->setRelativeSpatialAcceleration(acceleration);

  auto cloned = original->clone(Frame::World());

  EXPECT_TRUE(
      cloned->getSpatialVelocity().isApprox(original->getSpatialVelocity()));
  EXPECT_TRUE(cloned->getSpatialAcceleration().isApprox(
      original->getSpatialAcceleration()));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialVelocityInWorldCoordinates)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.linear()
      = Eigen::AngleAxisd(dart::math::pi / 2, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");

  Eigen::Vector6d velocityInWorld;
  velocityInWorld << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;

  child->setRelativeSpatialVelocity(velocityInWorld, Frame::World());

  // Velocity should be transformed from world to child's coordinates
  EXPECT_FALSE(child->getRelativeSpatialVelocity().isApprox(velocityInWorld));
}

//==============================================================================
TEST(SimpleFrameTest, SetRelativeSpatialAccelerationInWorldCoordinates)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.linear()
      = Eigen::AngleAxisd(dart::math::pi / 2, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");

  Eigen::Vector6d accelInWorld;
  accelInWorld << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;

  child->setRelativeSpatialAcceleration(accelInWorld, Frame::World());

  // Acceleration should be transformed from world to child's coordinates
  EXPECT_FALSE(child->getRelativeSpatialAcceleration().isApprox(accelInWorld));
}

//==============================================================================
TEST(SimpleFrameTest, SetTranslationWithRespectToFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");

  // Set translation in world coordinates
  Eigen::Vector3d worldTranslation(20.0, 5.0, 0.0);
  child->setTranslation(worldTranslation, Frame::World());

  EXPECT_TRUE(child->getWorldTransform().translation().isApprox(
      worldTranslation, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, SetRotationWithRespectToFrame)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.linear()
      = Eigen::AngleAxisd(dart::math::pi / 4, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");

  // Set rotation in world coordinates
  Eigen::Matrix3d worldRotation
      = Eigen::AngleAxisd(dart::math::pi / 2, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  child->setRotation(worldRotation, Frame::World());

  EXPECT_TRUE(
      child->getWorldTransform().linear().isApprox(worldRotation, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, CopyWithoutProperties)
{
  auto source = SimpleFrame::createShared(Frame::World(), "source_name");
  Eigen::Isometry3d sourceTf = Eigen::Isometry3d::Identity();
  sourceTf.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  source->setRelativeTransform(sourceTf);

  auto dest = SimpleFrame::createShared(Frame::World(), "dest_name");
  dest->copy(*source, Frame::World(), false);

  // Transform should be copied
  EXPECT_TRUE(dest->getTransform().isApprox(source->getTransform(), 1e-10));
  // Name should NOT be copied when _copyProperties is false
  EXPECT_EQ(dest->getName(), "dest_name");
}

//==============================================================================
TEST(SimpleFrameTest, CopyWithProperties)
{
  auto source = SimpleFrame::createShared(Frame::World(), "source_name");
  Eigen::Isometry3d sourceTf = Eigen::Isometry3d::Identity();
  sourceTf.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  source->setRelativeTransform(sourceTf);

  auto dest = SimpleFrame::createShared(Frame::World(), "dest_name");
  dest->copy(*source, Frame::World(), true);

  // Transform should be copied
  EXPECT_TRUE(dest->getTransform().isApprox(source->getTransform(), 1e-10));
  // Name should be copied when _copyProperties is true
  EXPECT_EQ(dest->getName(), "source_name");
}

//==============================================================================
TEST(SimpleFrameTest, CopyNullFrameDoesNothing)
{
  auto dest = SimpleFrame::createShared(Frame::World(), "dest");
  Eigen::Isometry3d originalTf = Eigen::Isometry3d::Identity();
  originalTf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  dest->setRelativeTransform(originalTf);

  // Copy from nullptr should do nothing
  dest->copy(static_cast<const Frame*>(nullptr), Frame::World());

  EXPECT_TRUE(dest->getRelativeTransform().isApprox(originalTf));
}

//==============================================================================
TEST(SimpleFrameTest, CopySelfDoesNothing)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");
  Eigen::Isometry3d originalTf = Eigen::Isometry3d::Identity();
  originalTf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  frame->setRelativeTransform(originalTf);

  // Copy self with same parent should do nothing
  frame->copy(*frame, Frame::World());

  EXPECT_TRUE(frame->getRelativeTransform().isApprox(originalTf));
}

//==============================================================================
TEST(SimpleFrameTest, WorldFrameTransformIsIdentity)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  // Frame at world origin should have identity world transform
  EXPECT_TRUE(
      frame->getWorldTransform().isApprox(Eigen::Isometry3d::Identity()));
}

//==============================================================================
TEST(SimpleFrameTest, NestedFrameWorldTransform)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Isometry3d parentTf = Eigen::Isometry3d::Identity();
  parentTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  parentTf.linear()
      = Eigen::AngleAxisd(dart::math::pi / 2, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  parent->setRelativeTransform(parentTf);

  auto child = SimpleFrame::createShared(parent.get(), "child");
  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  child->setRelativeTransform(childTf);

  // World transform should be parent * child
  Eigen::Isometry3d expectedWorld = parentTf * childTf;
  EXPECT_TRUE(child->getWorldTransform().isApprox(expectedWorld, 1e-10));

  // In world coordinates, child should be at (1, 1, 0) due to rotation
  Eigen::Vector3d expectedWorldPos(1.0, 1.0, 0.0);
  EXPECT_TRUE(child->getWorldTransform().translation().isApprox(
      expectedWorldPos, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, GetTransformBetweenFrames)
{
  auto frame1 = SimpleFrame::createShared(Frame::World(), "frame1");
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  frame1->setRelativeTransform(tf1);

  auto frame2 = SimpleFrame::createShared(Frame::World(), "frame2");
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  frame2->setRelativeTransform(tf2);

  // Transform from frame1 to frame2
  Eigen::Isometry3d tf1to2 = frame2->getTransform(frame1.get());
  Eigen::Vector3d expectedTranslation(2.0, 0.0, 0.0);
  EXPECT_TRUE(tf1to2.translation().isApprox(expectedTranslation, 1e-10));

  // Transform from frame2 to frame1
  Eigen::Isometry3d tf2to1 = frame1->getTransform(frame2.get());
  Eigen::Vector3d expectedReverseTranslation(-2.0, 0.0, 0.0);
  EXPECT_TRUE(tf2to1.translation().isApprox(expectedReverseTranslation, 1e-10));
}

//==============================================================================
TEST(SimpleFrameTest, PartialAccelerationWithMovingParent)
{
  auto parent = SimpleFrame::createShared(Frame::World(), "parent");
  Eigen::Vector6d parentVel;
  parentVel << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0; // Angular velocity about Z
  parent->setRelativeSpatialVelocity(parentVel);

  auto child = SimpleFrame::createShared(parent.get(), "child");
  Eigen::Isometry3d childTf = Eigen::Isometry3d::Identity();
  childTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  child->setRelativeTransform(childTf);

  Eigen::Vector6d childVel;
  childVel << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0; // Linear velocity in X
  child->setRelativeSpatialVelocity(childVel);

  // Partial acceleration should be non-zero due to Coriolis effect
  const Eigen::Vector6d& partialAccel = child->getPartialAcceleration();
  EXPECT_FALSE(partialAccel.isZero());
}

//==============================================================================
TEST(SimpleFrameTest, CopyConstructor)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  tf.linear() = Eigen::AngleAxisd(dart::math::pi / 4, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();

  auto original = SimpleFrame::createShared(Frame::World(), "original", tf);

  Eigen::Vector6d velocity;
  velocity << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;
  original->setRelativeSpatialVelocity(velocity);

  // Use copy constructor via clone
  auto copied = original->clone(Frame::World());

  EXPECT_NE(copied.get(), original.get());
  EXPECT_TRUE(
      copied->getWorldTransform().isApprox(original->getWorldTransform()));
  EXPECT_TRUE(
      copied->getSpatialVelocity().isApprox(original->getSpatialVelocity()));
}

//==============================================================================
TEST(SimpleFrameTest, SetClassicDerivativesWithAcceleration)
{
  auto frame = SimpleFrame::createShared(Frame::World(), "frame");

  Eigen::Vector3d linearVel(1.0, 0.0, 0.0);
  Eigen::Vector3d angularVel(0.0, 0.0, 1.0);
  Eigen::Vector3d linearAcc(0.0, 1.0, 0.0);
  Eigen::Vector3d angularAcc(0.0, 0.0, 0.1);

  frame->setClassicDerivatives(linearVel, angularVel, linearAcc, angularAcc);

  // Spatial velocity should have angular part first, then linear
  Eigen::Vector6d expectedVel;
  expectedVel << angularVel, linearVel;
  EXPECT_TRUE(frame->getRelativeSpatialVelocity().isApprox(expectedVel, 1e-10));

  // Spatial acceleration includes w x v correction
  Eigen::Vector6d spatialAccel = frame->getRelativeSpatialAcceleration();
  EXPECT_FALSE(spatialAccel.isZero());
}
