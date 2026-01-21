// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/EllipsoidShape.hpp>

#include <dart/math/Constants.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(EllipsoidShapeTest, Construction)
{
  const Eigen::Vector3d diameters(2.0, 4.0, 6.0);
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(diameters);

  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().x(), 2.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().y(), 4.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().z(), 6.0);

  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().x(), 1.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().y(), 2.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().z(), 3.0);
}

//==============================================================================
TEST(EllipsoidShapeTest, SetDiameters)
{
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(2.0, 2.0, 2.0));

  ellipsoid->setDiameters(Eigen::Vector3d(4.0, 6.0, 8.0));
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().x(), 4.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().y(), 6.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().z(), 8.0);
}

//==============================================================================
TEST(EllipsoidShapeTest, SetRadii)
{
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(2.0, 2.0, 2.0));

  ellipsoid->setRadii(Eigen::Vector3d(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().x(), 2.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().y(), 3.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getRadii().z(), 4.0);

  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().x(), 4.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().y(), 6.0);
  EXPECT_DOUBLE_EQ(ellipsoid->getDiameters().z(), 8.0);
}

//==============================================================================
TEST(EllipsoidShapeTest, StaticType)
{
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(2.0, 2.0, 2.0));
  EXPECT_EQ(ellipsoid->getType(), "EllipsoidShape");
  EXPECT_EQ(dart::dynamics::EllipsoidShape::getStaticType(), "EllipsoidShape");
}

//==============================================================================
TEST(EllipsoidShapeTest, IsSphere)
{
  auto sphere = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(2.0, 2.0, 2.0));
  EXPECT_TRUE(sphere->isSphere());

  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(2.0, 4.0, 6.0));
  EXPECT_FALSE(ellipsoid->isSphere());
}

//==============================================================================
TEST(EllipsoidShapeTest, Volume)
{
  const Eigen::Vector3d diameters(2.0, 4.0, 6.0);
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(diameters);

  // V = (4/3) * pi * rx * ry * rz
  const Eigen::Vector3d radii = diameters / 2.0;
  const double expectedVolume
      = (4.0 / 3.0) * dart::math::pi * radii.x() * radii.y() * radii.z();

  EXPECT_NEAR(ellipsoid->getVolume(), expectedVolume, 1e-10);
}

//==============================================================================
TEST(EllipsoidShapeTest, BoundingBox)
{
  const Eigen::Vector3d diameters(2.0, 4.0, 6.0);
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(diameters);
  const auto& bbox = ellipsoid->getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -1.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -2.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().z(), -3.0);

  EXPECT_DOUBLE_EQ(bbox.getMax().x(), 1.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().z(), 3.0);
}

//==============================================================================
TEST(EllipsoidShapeTest, Clone)
{
  const Eigen::Vector3d diameters(2.0, 4.0, 6.0);
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(diameters);
  auto clone = ellipsoid->clone();

  ASSERT_NE(clone, nullptr);

  auto ellipsoidClone
      = std::dynamic_pointer_cast<dart::dynamics::EllipsoidShape>(clone);
  ASSERT_NE(ellipsoidClone, nullptr);

  EXPECT_TRUE(ellipsoidClone->getDiameters().isApprox(diameters));
}

//==============================================================================
TEST(EllipsoidShapeTest, Inertia)
{
  const Eigen::Vector3d diameters(2.0, 4.0, 6.0);
  const double mass = 1.0;

  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(diameters);
  auto inertia = ellipsoid->computeInertia(mass);

  EXPECT_NEAR(inertia(0, 1), 0.0, 1e-10);
  EXPECT_NEAR(inertia(0, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 1), 0.0, 1e-10);

  EXPECT_GT(inertia(0, 0), 0.0);
  EXPECT_GT(inertia(1, 1), 0.0);
  EXPECT_GT(inertia(2, 2), 0.0);
}

//==============================================================================
TEST(EllipsoidShapeTest, SmallDimensions)
{
  auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(1e-10, 1e-10, 1e-10));
  EXPECT_GT(ellipsoid->getDiameters().x(), 0.0);
  EXPECT_GT(ellipsoid->getDiameters().y(), 0.0);
  EXPECT_GT(ellipsoid->getDiameters().z(), 0.0);
  EXPECT_GT(ellipsoid->getVolume(), 0.0);
}
