// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/pyramid_shape.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(PyramidShapeTest, Construction)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(pyramid->getBaseWidth(), 1.0);
  EXPECT_DOUBLE_EQ(pyramid->getBaseDepth(), 2.0);
  EXPECT_DOUBLE_EQ(pyramid->getHeight(), 3.0);
}

//==============================================================================
TEST(PyramidShapeTest, SetBaseWidth)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);

  pyramid->setBaseWidth(5.0);
  EXPECT_DOUBLE_EQ(pyramid->getBaseWidth(), 5.0);

  pyramid->setBaseWidth(0.001);
  EXPECT_DOUBLE_EQ(pyramid->getBaseWidth(), 0.001);
}

//==============================================================================
TEST(PyramidShapeTest, SetBaseDepth)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);

  pyramid->setBaseDepth(4.0);
  EXPECT_DOUBLE_EQ(pyramid->getBaseDepth(), 4.0);

  pyramid->setBaseDepth(0.001);
  EXPECT_DOUBLE_EQ(pyramid->getBaseDepth(), 0.001);
}

//==============================================================================
TEST(PyramidShapeTest, SetHeight)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);

  pyramid->setHeight(6.0);
  EXPECT_DOUBLE_EQ(pyramid->getHeight(), 6.0);

  pyramid->setHeight(0.001);
  EXPECT_DOUBLE_EQ(pyramid->getHeight(), 0.001);
}

//==============================================================================
TEST(PyramidShapeTest, StaticType)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);
  EXPECT_EQ(pyramid->getType(), "PyramidShape");
  EXPECT_EQ(dart::dynamics::PyramidShape::getStaticType(), "PyramidShape");
}

//==============================================================================
TEST(PyramidShapeTest, Volume)
{
  const double baseWidth = 3.0;
  const double baseDepth = 4.0;
  const double height = 6.0;

  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(
      baseWidth, baseDepth, height);

  const double expectedVolume = (1.0 / 3.0) * baseWidth * baseDepth * height;
  EXPECT_NEAR(pyramid->getVolume(), expectedVolume, 1e-10);

  EXPECT_NEAR(
      dart::dynamics::PyramidShape::computeVolume(baseWidth, baseDepth, height),
      expectedVolume,
      1e-10);
}

//==============================================================================
TEST(PyramidShapeTest, BoundingBox)
{
  const double baseWidth = 2.0;
  const double baseDepth = 4.0;
  const double height = 6.0;

  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(
      baseWidth, baseDepth, height);
  const auto& bbox = pyramid->getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox.getMin().x(), -baseWidth / 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().y(), -baseDepth / 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMin().z(), -height / 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().x(), baseWidth / 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().y(), baseDepth / 2.0);
  EXPECT_DOUBLE_EQ(bbox.getMax().z(), height / 2.0);
}

//==============================================================================
TEST(PyramidShapeTest, Clone)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);
  auto clone = pyramid->clone();

  ASSERT_NE(clone, nullptr);

  auto pyramidClone
      = std::dynamic_pointer_cast<dart::dynamics::PyramidShape>(clone);
  ASSERT_NE(pyramidClone, nullptr);

  EXPECT_DOUBLE_EQ(pyramidClone->getBaseWidth(), 1.0);
  EXPECT_DOUBLE_EQ(pyramidClone->getBaseDepth(), 2.0);
  EXPECT_DOUBLE_EQ(pyramidClone->getHeight(), 3.0);
}

//==============================================================================
TEST(PyramidShapeTest, Inertia)
{
  const double mass = 1.0;

  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 2.0, 3.0);
  auto inertia = pyramid->computeInertia(mass);

  EXPECT_EQ(inertia, Eigen::Matrix3d::Identity());
}

//==============================================================================
TEST(PyramidShapeTest, VolumeAfterDimensionChange)
{
  auto pyramid = std::make_shared<dart::dynamics::PyramidShape>(1.0, 1.0, 1.0);

  const double v1 = pyramid->getVolume();

  pyramid->setBaseWidth(2.0);
  const double v2 = pyramid->getVolume();
  EXPECT_GT(v2, v1);

  pyramid->setBaseDepth(2.0);
  const double v3 = pyramid->getVolume();
  EXPECT_GT(v3, v2);

  pyramid->setHeight(2.0);
  const double v4 = pyramid->getVolume();
  EXPECT_GT(v4, v3);

  const double expectedVolume = (1.0 / 3.0) * 2.0 * 2.0 * 2.0;
  EXPECT_NEAR(v4, expectedVolume, 1e-10);
}
