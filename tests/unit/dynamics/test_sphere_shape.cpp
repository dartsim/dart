// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/sphere_shape.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

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
