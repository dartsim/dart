// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/cone_shape.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(ConeShapeTest, Construction)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(0.5, 2.0);
  EXPECT_DOUBLE_EQ(cone->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(cone->getHeight(), 2.0);
}

//==============================================================================
TEST(ConeShapeTest, SetRadius)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(0.5, 2.0);

  cone->setRadius(1.0);
  EXPECT_DOUBLE_EQ(cone->getRadius(), 1.0);

  cone->setRadius(0.001);
  EXPECT_DOUBLE_EQ(cone->getRadius(), 0.001);

  cone->setRadius(100.0);
  EXPECT_DOUBLE_EQ(cone->getRadius(), 100.0);
}

//==============================================================================
TEST(ConeShapeTest, SetHeight)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(0.5, 2.0);

  cone->setHeight(3.0);
  EXPECT_DOUBLE_EQ(cone->getHeight(), 3.0);

  cone->setHeight(0.001);
  EXPECT_DOUBLE_EQ(cone->getHeight(), 0.001);

  cone->setHeight(100.0);
  EXPECT_DOUBLE_EQ(cone->getHeight(), 100.0);
}

//==============================================================================
TEST(ConeShapeTest, StaticType)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(0.5, 2.0);
  EXPECT_EQ(cone->getType(), "ConeShape");
  EXPECT_EQ(dart::dynamics::ConeShape::getStaticType(), "ConeShape");
}

//==============================================================================
TEST(ConeShapeTest, Volume)
{
  const double radius = 1.0;
  const double height = 3.0;

  auto cone = std::make_shared<dart::dynamics::ConeShape>(radius, height);

  // V = (1/3) * pi * r^2 * h
  const double expectedVolume
      = (1.0 / 3.0) * dart::math::pi * radius * radius * height;

  EXPECT_NEAR(cone->getVolume(), expectedVolume, 1e-10);
}

//==============================================================================
TEST(ConeShapeTest, BoundingBox)
{
  const double radius = 1.0;
  const double height = 2.0;

  auto cone = std::make_shared<dart::dynamics::ConeShape>(radius, height);
  const auto& bbox = cone->getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().x(), radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), radius);
}

//==============================================================================
TEST(ConeShapeTest, Clone)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(0.5, 2.0);
  auto clone = cone->clone();

  ASSERT_NE(clone, nullptr);

  auto coneClone = std::dynamic_pointer_cast<dart::dynamics::ConeShape>(clone);
  ASSERT_NE(coneClone, nullptr);

  EXPECT_DOUBLE_EQ(coneClone->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(coneClone->getHeight(), 2.0);
}

//==============================================================================
TEST(ConeShapeTest, Inertia)
{
  const double radius = 1.0;
  const double height = 2.0;
  const double mass = 1.0;

  auto cone = std::make_shared<dart::dynamics::ConeShape>(radius, height);
  auto inertia = cone->computeInertia(mass);

  EXPECT_NEAR(inertia(0, 1), 0.0, 1e-10);
  EXPECT_NEAR(inertia(0, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 1), 0.0, 1e-10);

  EXPECT_NEAR(inertia(0, 0), inertia(1, 1), 1e-10);

  EXPECT_GT(inertia(0, 0), 0.0);
  EXPECT_GT(inertia(1, 1), 0.0);
  EXPECT_GT(inertia(2, 2), 0.0);
}

//==============================================================================
TEST(ConeShapeTest, SmallDimensions)
{
  auto cone = std::make_shared<dart::dynamics::ConeShape>(1e-10, 1e-10);
  EXPECT_GT(cone->getRadius(), 0.0);
  EXPECT_GT(cone->getHeight(), 0.0);
  EXPECT_GT(cone->getVolume(), 0.0);
}
