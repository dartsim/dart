// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/cylinder_shape.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(CylinderShapeTest, Construction)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 2.0);
}

//==============================================================================
TEST(CylinderShapeTest, SetRadius)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  cylinder->setRadius(1.0);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 1.0);

  cylinder->setRadius(0.001);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 0.001);

  cylinder->setRadius(100.0);
  EXPECT_DOUBLE_EQ(cylinder->getRadius(), 100.0);
}

//==============================================================================
TEST(CylinderShapeTest, SetHeight)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  cylinder->setHeight(3.0);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 3.0);

  cylinder->setHeight(0.001);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 0.001);

  cylinder->setHeight(100.0);
  EXPECT_DOUBLE_EQ(cylinder->getHeight(), 100.0);
}

//==============================================================================
TEST(CylinderShapeTest, StaticType)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  EXPECT_EQ(cylinder->getType(), "CylinderShape");
  EXPECT_EQ(dart::dynamics::CylinderShape::getStaticType(), "CylinderShape");
}

//==============================================================================
TEST(CylinderShapeTest, Volume)
{
  const double radius = 1.0;
  const double height = 3.0;

  auto cylinder
      = std::make_shared<dart::dynamics::CylinderShape>(radius, height);

  // V = pi * r^2 * h
  const double expectedVolume = dart::math::pi * radius * radius * height;

  EXPECT_NEAR(cylinder->getVolume(), expectedVolume, 1e-10);
}

//==============================================================================
TEST(CylinderShapeTest, BoundingBox)
{
  const double radius = 1.0;
  const double height = 2.0;

  auto cylinder
      = std::make_shared<dart::dynamics::CylinderShape>(radius, height);
  const auto& bbox = cylinder->getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMin().z(), -height / 2.0);

  EXPECT_DOUBLE_EQ(bbox.getMax().x(), radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().z(), height / 2.0);
}

//==============================================================================
TEST(CylinderShapeTest, Clone)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto clone = cylinder->clone();

  ASSERT_NE(clone, nullptr);

  auto cylinderClone
      = std::dynamic_pointer_cast<dart::dynamics::CylinderShape>(clone);
  ASSERT_NE(cylinderClone, nullptr);

  EXPECT_DOUBLE_EQ(cylinderClone->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(cylinderClone->getHeight(), 2.0);
}

//==============================================================================
TEST(CylinderShapeTest, Inertia)
{
  const double radius = 1.0;
  const double height = 2.0;
  const double mass = 1.0;

  auto cylinder
      = std::make_shared<dart::dynamics::CylinderShape>(radius, height);
  auto inertia = cylinder->computeInertia(mass);

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
TEST(CylinderShapeTest, SmallDimensions)
{
  auto cylinder = std::make_shared<dart::dynamics::CylinderShape>(1e-10, 1e-10);
  EXPECT_GT(cylinder->getRadius(), 0.0);
  EXPECT_GT(cylinder->getHeight(), 0.0);
  EXPECT_GT(cylinder->getVolume(), 0.0);
}
