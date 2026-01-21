// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/LineSegmentShape.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace dart;
using namespace dart::dynamics;

TEST(LineSegmentShape, ThicknessValidation)
{
  LineSegmentShape shape(-1.0f);
  EXPECT_FLOAT_EQ(shape.getThickness(), 1.0f);

  shape.setThickness(0.0f);
  EXPECT_FLOAT_EQ(shape.getThickness(), 1.0f);

  shape.setThickness(0.25f);
  EXPECT_FLOAT_EQ(shape.getThickness(), 0.25f);
}

TEST(LineSegmentShape, VertexAndConnectionManagement)
{
  LineSegmentShape shape(0.1f);

  shape.removeVertex(1);
  shape.setVertex(0, Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_TRUE(shape.getVertex(0).isApprox(Eigen::Vector3d::Zero()));

  const auto v0 = Eigen::Vector3d(0.0, 0.0, 0.0);
  const auto v1 = Eigen::Vector3d(1.0, 0.0, 0.0);

  const auto idx0 = shape.addVertex(v0);
  const auto idx1 = shape.addVertex(v1, 10);

  EXPECT_EQ(idx0, 0u);
  EXPECT_EQ(idx1, 1u);
  EXPECT_EQ(shape.getVertices().size(), 2u);
  EXPECT_TRUE(shape.getConnections().empty());

  shape.addConnection(idx0, idx1);
  EXPECT_EQ(shape.getConnections().size(), 1u);

  shape.addConnection(10, 11);
  EXPECT_EQ(shape.getConnections().size(), 1u);

  shape.removeConnection(idx0, idx1);
  EXPECT_TRUE(shape.getConnections().empty());

  shape.removeConnection(0);
  EXPECT_TRUE(shape.getConnections().empty());
}

TEST(LineSegmentShape, InertiaBoundingBoxAndClone)
{
  LineSegmentShape shape(
      Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), 0.5f);

  const auto inertia = shape.computeInertia(2.0);
  EXPECT_TRUE(inertia.isApprox(inertia.transpose()));
  EXPECT_GT(inertia(0, 0), 0.0);

  const auto& box = shape.getBoundingBox();
  EXPECT_TRUE(box.getMin().isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(box.getMax().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));

  EXPECT_DOUBLE_EQ(shape.getVolume(), 0.0);

  auto cloned = shape.clone();
  auto* clonedLine = dynamic_cast<LineSegmentShape*>(cloned.get());
  ASSERT_NE(clonedLine, nullptr);
  EXPECT_EQ(clonedLine->getVertices().size(), 2u);
  EXPECT_EQ(clonedLine->getConnections().size(), 1u);
}
