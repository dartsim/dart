// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/heightmap_shape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

//==============================================================================
TEST(HeightmapShape, BoundingBoxMatchesSampleSpacing)
{
  dart::dynamics::HeightmapShaped shape;
  std::vector<double> heights = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

  shape.setHeightField(3u, 2u, heights);
  shape.setScale(Eigen::Vector3d(0.5, 0.25, 1.5));

  const auto bbox = shape.getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox.getMin()[0], -0.75);
  EXPECT_DOUBLE_EQ(bbox.getMax()[0], 0.75);
  EXPECT_DOUBLE_EQ(bbox.getMin()[1], -0.25);
  EXPECT_DOUBLE_EQ(bbox.getMax()[1], 0.25);
  EXPECT_DOUBLE_EQ(bbox.getMin()[2], 1.5);
  EXPECT_DOUBLE_EQ(bbox.getMax()[2], 9.0);
}

//==============================================================================
TEST(HeightmapShape, AccessorsCloneVolumeAndFlip)
{
  dart::dynamics::HeightmapShaped shape;
  std::vector<double> heights = {1.0, 2.0, 3.0, 4.0};

  shape.setHeightField(2u, 2u, heights);
  shape.setScale(Eigen::Vector3d(0.5, 0.25, 2.0));

  EXPECT_TRUE(shape.getScale().isApprox(Eigen::Vector3d(0.5, 0.25, 2.0)));
  EXPECT_EQ(shape.getWidth(), 2u);
  EXPECT_EQ(shape.getDepth(), 2u);
  EXPECT_DOUBLE_EQ(shape.getMinHeight(), 1.0);
  EXPECT_DOUBLE_EQ(shape.getMaxHeight(), 4.0);
  EXPECT_EQ(shape.getHeightField().rows(), 2);
  EXPECT_EQ(shape.getHeightField().cols(), 2);

  shape.getHeightFieldModifiable()(0, 0) = 5.0;
  EXPECT_DOUBLE_EQ(shape.getHeightField()(0, 0), 5.0);
  shape.flipY();
  EXPECT_DOUBLE_EQ(shape.getHeightField()(1, 0), 5.0);

  shape.notifyColorUpdated(Eigen::Vector4d::Ones());
  EXPECT_GT(shape.getVolume(), 0.0);
  EXPECT_TRUE(shape.computeInertia(2.0).array().isFinite().all());

  auto clone = std::dynamic_pointer_cast<dart::dynamics::HeightmapShaped>(
      shape.clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_TRUE(clone->getScale().isApprox(shape.getScale()));
  EXPECT_EQ(clone->getWidth(), shape.getWidth());
  EXPECT_EQ(clone->getDepth(), shape.getDepth());
}

//==============================================================================
TEST(HeightmapShape, FloatHeightFieldAccessors)
{
  dart::dynamics::HeightmapShapef shape;
  const std::vector<float> heights = {0.0f, 1.0f, 2.0f, 3.0f};

  shape.setHeightField(2u, 2u, heights);
  EXPECT_EQ(shape.getWidth(), 2u);
  EXPECT_EQ(shape.getDepth(), 2u);
  EXPECT_FLOAT_EQ(shape.getMinHeight(), 0.0f);
  EXPECT_FLOAT_EQ(shape.getMaxHeight(), 3.0f);
  EXPECT_GT(shape.getVolume(), 0.0);
}
