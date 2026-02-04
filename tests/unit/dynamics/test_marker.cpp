// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/marker.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <dart/common/platform.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dynamics;

class MarkerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    skeleton = Skeleton::create("test_skeleton");
    auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
    body = pair.second;
    body->setName("body");
  }

  SkeletonPtr skeleton;
  BodyNode* body;
};

TEST_F(MarkerTest, CreateMarker)
{
  EXPECT_EQ(body->getNumMarkers(), 0u);

  Marker* marker = body->createMarker(std::string("test_marker"));
  ASSERT_NE(marker, nullptr);

  EXPECT_EQ(body->getNumMarkers(), 1u);
  EXPECT_EQ(marker->getName(), "test_marker");
  EXPECT_EQ(marker->getBodyNodePtr().get(), body);
}

TEST_F(MarkerTest, CreateMultipleMarkers)
{
  Marker* m1 = body->createMarker(std::string("marker1"));
  Marker* m2 = body->createMarker(std::string("marker2"));
  Marker* m3 = body->createMarker(std::string("marker3"));

  EXPECT_EQ(body->getNumMarkers(), 3u);
  EXPECT_EQ(body->getMarker(0), m1);
  EXPECT_EQ(body->getMarker(1), m2);
  EXPECT_EQ(body->getMarker(2), m3);
}

TEST_F(MarkerTest, LocalPosition)
{
  Marker* marker = body->createMarker(std::string("test_marker"));

  EXPECT_TRUE(marker->getLocalPosition().isApprox(Eigen::Vector3d::Zero()));

  Eigen::Vector3d newPos(0.1, 0.2, 0.3);
  marker->setLocalPosition(newPos);
  EXPECT_TRUE(marker->getLocalPosition().isApprox(newPos));
}

TEST_F(MarkerTest, WorldPosition)
{
  Marker* marker = body->createMarker(std::string("test_marker"));

  Eigen::Vector3d localPos(0.5, 0.0, 0.0);
  marker->setLocalPosition(localPos);

  Eigen::Isometry3d bodyTf = Eigen::Isometry3d::Identity();
  bodyTf.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  skeleton->getJoint(0)->setPositions(FreeJoint::convertToPositions(bodyTf));

  Eigen::Vector3d expectedWorld = bodyTf * localPos;
  EXPECT_TRUE(marker->getWorldPosition().isApprox(expectedWorld, 1e-10));
}

TEST_F(MarkerTest, UniqueID)
{
  Marker* m1 = body->createMarker(std::string("marker1"));
  Marker* m2 = body->createMarker(std::string("marker2"));

  EXPECT_NE(m1->getID(), m2->getID());
}

TEST_F(MarkerTest, ConstraintType)
{
  Marker* marker = body->createMarker(std::string("test_marker"));

  EXPECT_EQ(marker->getConstraintType(), Marker::NO);

  marker->setConstraintType(Marker::HARD);
  EXPECT_EQ(marker->getConstraintType(), Marker::HARD);

  marker->setConstraintType(Marker::SOFT);
  EXPECT_EQ(marker->getConstraintType(), Marker::SOFT);

  marker->setConstraintType(Marker::NO);
  EXPECT_EQ(marker->getConstraintType(), Marker::NO);
}

TEST_F(MarkerTest, Color)
{
  Marker* marker = body->createMarker(std::string("test_marker"));

  Eigen::Vector4d redColor(1.0, 0.0, 0.0, 1.0);
  marker->setColor(redColor);

  EXPECT_TRUE(marker->getColor().isApprox(redColor));

  Eigen::Vector4d greenSemiTransparent(0.0, 1.0, 0.0, 0.5);
  marker->setColor(greenSemiTransparent);
  EXPECT_TRUE(marker->getColor().isApprox(greenSemiTransparent));
}

TEST_F(MarkerTest, GetByIndex)
{
  body->createMarker(std::string("m1"));
  body->createMarker(std::string("m2"));

  EXPECT_EQ(body->getNumMarkers(), 2u);
  EXPECT_NE(body->getMarker(0), nullptr);
  EXPECT_NE(body->getMarker(1), nullptr);
}

TEST_F(MarkerTest, GetByName)
{
  body->createMarker(std::string("unique_marker"));

  EXPECT_NE(skeleton->getMarker("unique_marker"), nullptr);
  EXPECT_EQ(skeleton->getMarker("nonexistent"), nullptr);
}

TEST_F(MarkerTest, TransformWithRotation)
{
  Marker* marker = body->createMarker(std::string("test_marker"));

  Eigen::Vector3d localPos(1.0, 0.0, 0.0);
  marker->setLocalPosition(localPos);

  Eigen::Isometry3d bodyTf = Eigen::Isometry3d::Identity();
  bodyTf.linear() = Eigen::AngleAxisd(math::pi / 2, Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();
  skeleton->getJoint(0)->setPositions(FreeJoint::convertToPositions(bodyTf));

  Eigen::Vector3d expectedWorld = bodyTf * localPos;
  EXPECT_TRUE(marker->getWorldPosition().isApprox(expectedWorld, 1e-10));
}

TEST_F(MarkerTest, JacobianComputation)
{
  Marker* marker = body->createMarker(std::string("test_marker"));
  marker->setLocalPosition(Eigen::Vector3d(0.1, 0.0, 0.0));

  math::Jacobian J = marker->getJacobian();

  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), static_cast<int>(skeleton->getNumDofs()));
}

TEST_F(MarkerTest, WorldPositionAfterMove)
{
  Marker* marker = body->createMarker(std::string("test_marker"));
  marker->setLocalPosition(Eigen::Vector3d(0.0, 0.0, 0.5));

  Eigen::Vector3d initialWorld = marker->getWorldPosition();

  Eigen::Isometry3d newTf = Eigen::Isometry3d::Identity();
  newTf.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  skeleton->getJoint(0)->setPositions(FreeJoint::convertToPositions(newTf));

  Eigen::Vector3d newWorld = marker->getWorldPosition();
  EXPECT_FALSE(newWorld.isApprox(initialWorld));
  EXPECT_TRUE(newWorld.isApprox(Eigen::Vector3d(5.0, 5.0, 5.5), 1e-10));
}

TEST_F(MarkerTest, FixedFrameRelativeMotionIsZero)
{
  Marker* marker = body->createMarker(std::string("fixed_frame"));

  EXPECT_TRUE(marker->getRelativeSpatialVelocity().isZero(0.0));
  EXPECT_TRUE(marker->getRelativeSpatialAcceleration().isZero(0.0));
  EXPECT_TRUE(marker->getPrimaryRelativeAcceleration().isZero(0.0));
  EXPECT_TRUE(marker->getPartialAcceleration().isZero(0.0));

  FixedFrame::AspectProperties props;
  props.mRelativeTf = Eigen::Isometry3d::Identity();
  props.mRelativeTf.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  marker->FixedFrame::setAspectProperties(props);
  EXPECT_TRUE(marker->getRelativeTransform().translation().isApprox(
      Eigen::Vector3d(0.2, -0.1, 0.3)));
}

#if !DART_OS_WINDOWS
TEST_F(MarkerTest, FixedJacobianNodeDependencyAccessors)
{
  Marker* marker = body->createMarker(std::string("jacobian_node"));

  const std::size_t numDeps = marker->getNumDependentGenCoords();
  EXPECT_GT(numDeps, 0u);
  EXPECT_EQ(marker->getDependentGenCoordIndices().size(), numDeps);

  const std::size_t firstIndex = marker->getDependentGenCoordIndex(0);
  EXPECT_LT(firstIndex, skeleton->getNumDofs());
  EXPECT_TRUE(marker->dependsOn(firstIndex));

  EXPECT_EQ(marker->getNumDependentDofs(), marker->getDependentDofs().size());
  EXPECT_EQ(marker->getDependentDof(0), marker->getDependentDofs()[0]);

  const auto chainDofs = marker->getChainDofs();
  EXPECT_FALSE(chainDofs.empty());
}
#endif

TEST_F(MarkerTest, FixedJacobianNodeCachesUpdate)
{
  Marker* marker = body->createMarker(std::string("jacobian_cache"));

  const auto initialJacobian = marker->getJacobian();

  Eigen::Isometry3d newRelative = Eigen::Isometry3d::Identity();
  newRelative.translation() = Eigen::Vector3d(0.3, 0.2, -0.1);
  marker->setRelativeTransform(newRelative);

  const auto updatedJacobian = marker->getJacobian();
  EXPECT_FALSE(updatedJacobian.isApprox(initialJacobian));

  EXPECT_EQ(marker->getWorldJacobian().rows(), 6);
  EXPECT_EQ(marker->getWorldJacobian().cols(), initialJacobian.cols());
  EXPECT_EQ(marker->getJacobianSpatialDeriv().rows(), 6);
  EXPECT_EQ(marker->getJacobianClassicDeriv().rows(), 6);
}
