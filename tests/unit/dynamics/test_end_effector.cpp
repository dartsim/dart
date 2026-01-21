// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dynamics;

class EndEffectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    skeleton = Skeleton::create("test_skeleton");

    auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
    rootBody = pair.second;
    rootBody->setName("root");

    RevoluteJoint::Properties props;
    props.mName = "joint1";
    props.mAxis = Eigen::Vector3d::UnitZ();
    auto pair2
        = rootBody->createChildJointAndBodyNodePair<RevoluteJoint>(props);
    childBody = pair2.second;
    childBody->setName("child");
  }

  SkeletonPtr skeleton;
  BodyNode* rootBody;
  BodyNode* childBody;
};

TEST_F(EndEffectorTest, CreateEndEffector)
{
  EXPECT_EQ(skeleton->getNumEndEffectors(), 0u);

  EndEffector* ee = childBody->createEndEffector("test_ee");
  ASSERT_NE(ee, nullptr);

  EXPECT_EQ(skeleton->getNumEndEffectors(), 1u);
  EXPECT_EQ(ee->getName(), "test_ee");
  EXPECT_EQ(ee->getBodyNodePtr().get(), childBody);
}

TEST_F(EndEffectorTest, CreateMultipleEndEffectors)
{
  EndEffector* ee1 = childBody->createEndEffector("ee1");
  EndEffector* ee2 = childBody->createEndEffector("ee2");
  EndEffector* ee3 = rootBody->createEndEffector("ee_root");

  EXPECT_EQ(skeleton->getNumEndEffectors(), 3u);
  EXPECT_EQ(skeleton->getEndEffector(0), ee1);
  EXPECT_EQ(skeleton->getEndEffector(1), ee2);
  EXPECT_EQ(skeleton->getEndEffector(2), ee3);
  EXPECT_EQ(skeleton->getEndEffector("ee1"), ee1);
  EXPECT_EQ(skeleton->getEndEffector("ee2"), ee2);
  EXPECT_EQ(skeleton->getEndEffector("ee_root"), ee3);
}

TEST_F(EndEffectorTest, DefaultRelativeTransform)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  Eigen::Isometry3d defaultTf = ee->getRelativeTransform();
  EXPECT_TRUE(
      defaultTf.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()));

  Eigen::Isometry3d newTf = Eigen::Isometry3d::Identity();
  newTf.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  ee->setDefaultRelativeTransform(newTf, true);

  EXPECT_TRUE(ee->getRelativeTransform().matrix().isApprox(newTf.matrix()));
}

TEST_F(EndEffectorTest, ResetRelativeTransform)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  Eigen::Isometry3d defaultTf = Eigen::Isometry3d::Identity();
  defaultTf.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  ee->setDefaultRelativeTransform(defaultTf, false);

  Eigen::Isometry3d modifiedTf = Eigen::Isometry3d::Identity();
  modifiedTf.translation() = Eigen::Vector3d(1.0, 1.0, 1.0);
  ee->setRelativeTransform(modifiedTf);

  EXPECT_TRUE(
      ee->getRelativeTransform().matrix().isApprox(modifiedTf.matrix()));

  ee->resetRelativeTransform();
  EXPECT_TRUE(ee->getRelativeTransform().matrix().isApprox(defaultTf.matrix()));
}

TEST_F(EndEffectorTest, SupportAspect)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  EXPECT_FALSE(ee->hasSupport());

  Support* support = ee->getSupport(true);
  ASSERT_NE(support, nullptr);
  EXPECT_TRUE(ee->hasSupport());

  EXPECT_FALSE(support->isActive());

  support->setActive(true);
  EXPECT_TRUE(support->isActive());

  support->setActive(false);
  EXPECT_FALSE(support->isActive());
}

TEST_F(EndEffectorTest, SupportGeometry)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");
  Support* support = ee->getSupport(true);

  math::SupportGeometry geometry;
  geometry.push_back(Eigen::Vector3d(0.1, 0.1, 0.0));
  geometry.push_back(Eigen::Vector3d(-0.1, 0.1, 0.0));
  geometry.push_back(Eigen::Vector3d(-0.1, -0.1, 0.0));
  geometry.push_back(Eigen::Vector3d(0.1, -0.1, 0.0));

  support->setGeometry(geometry);

  const math::SupportGeometry& retrievedGeometry = support->getGeometry();
  EXPECT_EQ(retrievedGeometry.size(), 4u);
  EXPECT_TRUE(retrievedGeometry[0].isApprox(Eigen::Vector3d(0.1, 0.1, 0.0)));
}

TEST_F(EndEffectorTest, WorldTransform)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  Eigen::Isometry3d eeTf = Eigen::Isometry3d::Identity();
  eeTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);
  ee->setDefaultRelativeTransform(eeTf, true);

  Eigen::Isometry3d rootTf = Eigen::Isometry3d::Identity();
  rootTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  skeleton->getJoint(0)->setPositions(FreeJoint::convertToPositions(rootTf));

  Eigen::Isometry3d worldTf = ee->getWorldTransform();
  EXPECT_FALSE(worldTf.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_F(EndEffectorTest, CopyEndEffector)
{
  EndEffector* ee1 = childBody->createEndEffector("ee1");

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  ee1->setDefaultRelativeTransform(tf, true);

  EndEffector* ee2 = childBody->createEndEffector("ee2");
  ee2->copy(*ee1);

  EXPECT_TRUE(ee2->getRelativeTransform().matrix().isApprox(tf.matrix()));
}

TEST_F(EndEffectorTest, GetByIndex)
{
  childBody->createEndEffector("ee1");
  childBody->createEndEffector("ee2");

  EXPECT_EQ(skeleton->getNumEndEffectors(), 2u);
  EXPECT_NE(skeleton->getEndEffector(0), nullptr);
  EXPECT_NE(skeleton->getEndEffector(1), nullptr);
}

TEST_F(EndEffectorTest, GetByName)
{
  childBody->createEndEffector("unique_ee");

  EXPECT_NE(skeleton->getEndEffector("unique_ee"), nullptr);
  EXPECT_EQ(skeleton->getEndEffector("nonexistent"), nullptr);
}

TEST_F(EndEffectorTest, JacobianComputation)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  skeleton->getJoint(1)->setPosition(0, 0.5);

  math::Jacobian J = ee->getJacobian();

  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), static_cast<int>(skeleton->getNumDofs()));
}

TEST_F(EndEffectorTest, LinearJacobian)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  math::LinearJacobian Jv = ee->getLinearJacobian();

  EXPECT_EQ(Jv.rows(), 3);
  EXPECT_EQ(Jv.cols(), static_cast<int>(skeleton->getNumDofs()));
}

TEST_F(EndEffectorTest, AngularJacobian)
{
  EndEffector* ee = childBody->createEndEffector("test_ee");

  math::AngularJacobian Jw = ee->getAngularJacobian();

  EXPECT_EQ(Jw.rows(), 3);
  EXPECT_EQ(Jw.cols(), static_cast<int>(skeleton->getNumDofs()));
}
