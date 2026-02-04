// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/capsule_shape.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

//==============================================================================
// Test basic construction and getters
TEST(CapsuleShapeTest, Construction)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 2.0);
}

//==============================================================================
// Test setters update values correctly
TEST(CapsuleShapeTest, SetRadius)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  capsule->setRadius(1.0);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 1.0);

  capsule->setRadius(0.001);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 0.001);

  capsule->setRadius(100.0);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 100.0);
}

//==============================================================================
TEST(CapsuleShapeTest, SetHeight)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  capsule->setHeight(3.0);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 3.0);

  capsule->setHeight(0.001);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 0.001);

  capsule->setHeight(100.0);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 100.0);
}

//==============================================================================
// Test static type
TEST(CapsuleShapeTest, StaticType)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  EXPECT_EQ(capsule->getType(), "CapsuleShape");
  EXPECT_EQ(dart::dynamics::CapsuleShape::getStaticType(), "CapsuleShape");
}

//==============================================================================
// Test volume computation
TEST(CapsuleShapeTest, Volume)
{
  const double radius = 1.0;
  const double height = 2.0;

  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(radius, height);

  // Volume = cylinder + sphere
  // Cylinder: pi * r^2 * h
  // Sphere: (4/3) * pi * r^3
  const double expectedVolume
      = dart::math::pi * radius * radius * height
        + (4.0 / 3.0) * dart::math::pi * radius * radius * radius;

  EXPECT_NEAR(capsule->getVolume(), expectedVolume, 1e-10);
}

//==============================================================================
// Test bounding box
TEST(CapsuleShapeTest, BoundingBox)
{
  const double radius = 1.0;
  const double height = 2.0;

  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(radius, height);
  const auto& bbox = capsule->getBoundingBox();

  // Bounding box should be: [-r, -r, -(r + h/2)] to [r, r, r + h/2]
  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -radius);
  EXPECT_DOUBLE_EQ(bbox.getMin().z(), -(radius + height / 2.0));

  EXPECT_DOUBLE_EQ(bbox.getMax().x(), radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), radius);
  EXPECT_DOUBLE_EQ(bbox.getMax().z(), radius + height / 2.0);
}

//==============================================================================
// Test clone
TEST(CapsuleShapeTest, Clone)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto clone = capsule->clone();

  ASSERT_NE(clone, nullptr);

  auto capsuleClone
      = std::dynamic_pointer_cast<dart::dynamics::CapsuleShape>(clone);
  ASSERT_NE(capsuleClone, nullptr);

  EXPECT_DOUBLE_EQ(capsuleClone->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(capsuleClone->getHeight(), 2.0);
}

//==============================================================================
// Test inertia computation
TEST(CapsuleShapeTest, Inertia)
{
  const double radius = 1.0;
  const double height = 2.0;
  const double mass = 1.0;

  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(radius, height);
  auto inertia = capsule->computeInertia(mass);

  // Inertia matrix should be diagonal
  EXPECT_NEAR(inertia(0, 1), 0.0, 1e-10);
  EXPECT_NEAR(inertia(0, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(1, 2), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 0), 0.0, 1e-10);
  EXPECT_NEAR(inertia(2, 1), 0.0, 1e-10);

  // Ixx == Iyy (rotational symmetry about z-axis)
  EXPECT_NEAR(inertia(0, 0), inertia(1, 1), 1e-10);

  // Diagonal elements should be positive
  EXPECT_GT(inertia(0, 0), 0.0);
  EXPECT_GT(inertia(1, 1), 0.0);
  EXPECT_GT(inertia(2, 2), 0.0);
}

//==============================================================================
TEST(CapsuleShapeTest, SmallDimensions)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(1e-10, 1e-10);
  EXPECT_GT(capsule->getRadius(), 0.0);
  EXPECT_GT(capsule->getHeight(), 0.0);
  EXPECT_GT(capsule->getVolume(), 0.0);
}

//==============================================================================
TEST(CapsuleShapeTest, VolumeUpdatesOnDimensionChange)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(1.0, 2.0);
  const double initialVolume = capsule->getVolume();

  capsule->setRadius(2.0);
  EXPECT_GT(capsule->getVolume(), initialVolume);

  capsule->setHeight(4.0);
  const double newVolume = capsule->getVolume();
  EXPECT_GT(newVolume, initialVolume);
}

//==============================================================================
TEST(CapsuleShapeTest, BoundingBoxUpdatesOnDimensionChange)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(1.0, 2.0);

  capsule->setRadius(2.0);
  capsule->setHeight(4.0);

  const auto& bbox = capsule->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -2.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -2.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().z(), -(2.0 + 4.0 / 2.0));

  EXPECT_DOUBLE_EQ(bbox.getMax().x(), 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().z(), 2.0 + 4.0 / 2.0);
}

//==============================================================================
TEST(CapsuleShapeTest, CloneIndependence)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto clone = std::dynamic_pointer_cast<dart::dynamics::CapsuleShape>(
      capsule->clone());

  capsule->setRadius(1.5);
  capsule->setHeight(5.0);

  EXPECT_DOUBLE_EQ(clone->getRadius(), 0.5);
  EXPECT_DOUBLE_EQ(clone->getHeight(), 2.0);
  EXPECT_DOUBLE_EQ(capsule->getRadius(), 1.5);
  EXPECT_DOUBLE_EQ(capsule->getHeight(), 5.0);
}

//==============================================================================
TEST(CapsuleShapeTest, InertiaWithDifferentMasses)
{
  auto capsule = std::make_shared<dart::dynamics::CapsuleShape>(1.0, 2.0);

  auto inertia1 = capsule->computeInertia(1.0);
  auto inertia2 = capsule->computeInertia(2.0);

  EXPECT_TRUE(inertia2.isApprox(2.0 * inertia1, 1e-10));
}
