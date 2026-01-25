// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/simple_frame.hpp>

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
  newTf.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())
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
      = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX())
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
