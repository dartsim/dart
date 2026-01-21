// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Marker.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/Constants.hpp>

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
