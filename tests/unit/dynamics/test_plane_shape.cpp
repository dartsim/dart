// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/plane_shape.hpp>

#include <gtest/gtest.h>

#include <cmath>

//==============================================================================
TEST(PlaneShapeTest, Construction)
{
  const Eigen::Vector3d normal(0.0, 0.0, 1.0);
  const double offset = 0.0;

  auto plane = std::make_shared<dart::dynamics::PlaneShape>(normal, offset);

  EXPECT_TRUE(plane->getNormal().isApprox(normal));
  EXPECT_DOUBLE_EQ(plane->getOffset(), offset);
}

//==============================================================================
TEST(PlaneShapeTest, SetNormal)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  plane->setNormal(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));

  plane->setNormal(Eigen::Vector3d(0.0, 1.0, 0.0));
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
}

//==============================================================================
TEST(PlaneShapeTest, SetOffset)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  plane->setOffset(1.0);
  EXPECT_DOUBLE_EQ(plane->getOffset(), 1.0);

  plane->setOffset(-5.0);
  EXPECT_DOUBLE_EQ(plane->getOffset(), -5.0);

  plane->setOffset(0.0);
  EXPECT_DOUBLE_EQ(plane->getOffset(), 0.0);
}

//==============================================================================
TEST(PlaneShapeTest, StaticType)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);
  EXPECT_EQ(plane->getType(), "PlaneShape");
  EXPECT_EQ(dart::dynamics::PlaneShape::getStaticType(), "PlaneShape");
}

//==============================================================================
TEST(PlaneShapeTest, NormalNormalized)
{
  const Eigen::Vector3d unnormalizedNormal(0.0, 0.0, 2.0);
  auto plane
      = std::make_shared<dart::dynamics::PlaneShape>(unnormalizedNormal, 0.0);

  EXPECT_NEAR(plane->getNormal().norm(), 1.0, 1e-10);

  plane->setNormal(Eigen::Vector3d(3.0, 4.0, 0.0));
  EXPECT_NEAR(plane->getNormal().norm(), 1.0, 1e-10);
}

//==============================================================================
TEST(PlaneShapeTest, Clone)
{
  const Eigen::Vector3d normal(0.0, 0.0, 1.0);
  const double offset = 2.5;

  auto plane = std::make_shared<dart::dynamics::PlaneShape>(normal, offset);
  auto clone = plane->clone();

  ASSERT_NE(clone, nullptr);

  auto planeClone
      = std::dynamic_pointer_cast<dart::dynamics::PlaneShape>(clone);
  ASSERT_NE(planeClone, nullptr);

  EXPECT_TRUE(planeClone->getNormal().isApprox(normal));
  EXPECT_DOUBLE_EQ(planeClone->getOffset(), offset);
}

//==============================================================================
TEST(PlaneShapeTest, SetNormalAndOffset)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  plane->setNormalAndOffset(Eigen::Vector3d(1.0, 0.0, 0.0), 3.0);
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(plane->getOffset(), 3.0);
}

//==============================================================================
TEST(PlaneShapeTest, NegativeOffset)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d(0.0, 0.0, 1.0), -10.0);

  EXPECT_DOUBLE_EQ(plane->getOffset(), -10.0);
}

//==============================================================================
TEST(PlaneShapeTest, DistancesAndBoundingBox)
{
  auto plane = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.5);

  EXPECT_DOUBLE_EQ(
      plane->computeSignedDistance(Eigen::Vector3d(0.0, 0.0, 1.0)), 0.5);
  EXPECT_DOUBLE_EQ(plane->computeDistance(Eigen::Vector3d(0.0, 0.0, 0.0)), 0.5);

  plane->setNormalAndPoint(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.0, 0.0, 2.0));
  EXPECT_DOUBLE_EQ(plane->getOffset(), 2.0);

  const auto& bbox = plane->getBoundingBox();
  EXPECT_TRUE(std::isinf(bbox.getMin()[0]));
  EXPECT_TRUE(std::isinf(bbox.getMax()[2]));
  EXPECT_DOUBLE_EQ(plane->getVolume(), 0.0);
}
