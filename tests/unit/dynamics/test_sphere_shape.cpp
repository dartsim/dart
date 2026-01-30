// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

#include <limits>

//==============================================================================
TEST(SphereShapeTest, ValidRadiusAccepted)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 1.0);

  sphere->setRadius(2.5);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 2.5);

  sphere->setRadius(0.001);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 0.001);
}

//==============================================================================
TEST(SphereShapeTest, NaNRadiusRejected)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);
  const double originalRadius = sphere->getRadius();

  sphere->setRadius(std::nan(""));
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);

  sphere->setRadius(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);

  sphere->setRadius(std::numeric_limits<double>::signaling_NaN());
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);
}

//==============================================================================
TEST(SphereShapeTest, InfiniteRadiusRejected)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);
  const double originalRadius = sphere->getRadius();

  sphere->setRadius(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);

  sphere->setRadius(-std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);
}

//==============================================================================
TEST(SphereShapeTest, NonPositiveRadiusRejected)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);
  const double originalRadius = sphere->getRadius();

  sphere->setRadius(0.0);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);

  sphere->setRadius(-1.0);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);

  sphere->setRadius(-0.001);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), originalRadius);
}

//==============================================================================
TEST(SphereShapeTest, ConstructorWithSmallValidRadius)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1e-10);
  EXPECT_GT(sphere->getRadius(), 0.0);
}

//==============================================================================
TEST(SphereShapeTest, StaticType)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);
  EXPECT_EQ(sphere->getType(), "SphereShape");
  EXPECT_EQ(dart::dynamics::SphereShape::getStaticType(), "SphereShape");
}

//==============================================================================
TEST(SphereShapeTest, ComputeVolume)
{
  const double radius = 2.0;
  // Volume = (4/3) * pi * r^3
  const double expectedVolume
      = (4.0 / 3.0) * dart::math::pi * radius * radius * radius;

  EXPECT_NEAR(
      dart::dynamics::SphereShape::computeVolume(radius),
      expectedVolume,
      1e-10);

  auto sphere = std::make_shared<dart::dynamics::SphereShape>(radius);
  EXPECT_NEAR(sphere->getVolume(), expectedVolume, 1e-10);
}

//==============================================================================
TEST(SphereShapeTest, ComputeInertia)
{
  const double radius = 1.5;
  const double mass = 3.0;

  // Inertia of solid sphere: I = (2/5) * m * r^2
  const double expectedI = (2.0 / 5.0) * mass * radius * radius;
  Eigen::Matrix3d expectedInertia = Eigen::Matrix3d::Zero();
  expectedInertia(0, 0) = expectedI;
  expectedInertia(1, 1) = expectedI;
  expectedInertia(2, 2) = expectedI;

  Eigen::Matrix3d inertia
      = dart::dynamics::SphereShape::computeInertia(radius, mass);
  EXPECT_TRUE(inertia.isApprox(expectedInertia, 1e-10));

  auto sphere = std::make_shared<dart::dynamics::SphereShape>(radius);
  Eigen::Matrix3d instanceInertia = sphere->computeInertia(mass);
  EXPECT_TRUE(instanceInertia.isApprox(expectedInertia, 1e-10));
}

//==============================================================================
TEST(SphereShapeTest, BoundingBox)
{
  const double radius = 2.5;
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(radius);

  const auto& bbox = sphere->getBoundingBox();
  EXPECT_TRUE(
      bbox.getMin().isApprox(Eigen::Vector3d(-radius, -radius, -radius)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(radius, radius, radius)));
}

//==============================================================================
TEST(SphereShapeTest, BoundingBoxUpdatesOnRadiusChange)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);

  sphere->setRadius(3.0);

  const auto& bbox = sphere->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-3.0, -3.0, -3.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(3.0, 3.0, 3.0)));
}

//==============================================================================
TEST(SphereShapeTest, Clone)
{
  const double radius = 1.75;
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(radius);

  auto cloned = sphere->clone();
  auto clonedSphere
      = std::dynamic_pointer_cast<dart::dynamics::SphereShape>(cloned);

  ASSERT_NE(clonedSphere, nullptr);
  EXPECT_DOUBLE_EQ(clonedSphere->getRadius(), radius);
  EXPECT_NE(clonedSphere.get(), sphere.get());
}

//==============================================================================
TEST(SphereShapeTest, UnitSphere)
{
  auto sphere = std::make_shared<dart::dynamics::SphereShape>(1.0);

  // Volume of unit sphere = (4/3) * pi
  const double expectedVolume = (4.0 / 3.0) * dart::math::pi;
  EXPECT_NEAR(sphere->getVolume(), expectedVolume, 1e-10);

  const auto& bbox = sphere->getBoundingBox();
  EXPECT_TRUE(bbox.getMin().isApprox(Eigen::Vector3d(-1.0, -1.0, -1.0)));
  EXPECT_TRUE(bbox.getMax().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
}
