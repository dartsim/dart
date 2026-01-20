// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/heightmap_shape.hpp>

#include <gtest/gtest.h>

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
