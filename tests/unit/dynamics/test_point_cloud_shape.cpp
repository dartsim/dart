// Copyright (c) 2011, The DART development contributors

#include "helpers/gtest_utils.hpp"

#include <dart/dynamics/point_cloud_shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::test;

namespace {

class ExposedPointCloudShape final : public PointCloudShape
{
public:
  using PointCloudShape::PointCloudShape;
  using PointCloudShape::updateBoundingBox;
  using PointCloudShape::updateVolume;
};

class ShapeHarness final : public Shape
{
public:
  ShapeHarness() : Shape(UNSUPPORTED)
  {
    // Do nothing
  }

  std::string_view getType() const override
  {
    return "ShapeHarness";
  }

  Eigen::Matrix3d computeInertia(double mass) const override
  {
    return Eigen::Matrix3d::Identity() * mass;
  }

  ShapePtr clone() const override
  {
    return std::make_shared<ShapeHarness>();
  }

protected:
  void updateVolume() const override
  {
    mVolume = 2.0;
    mIsVolumeDirty = false;
  }

  void updateBoundingBox() const override
  {
    mBoundingBox.setMin(Eigen::Vector3d(-1.0, -2.0, -3.0));
    mBoundingBox.setMax(Eigen::Vector3d(1.0, 2.0, 3.0));
    mIsBoundingBoxDirty = false;
  }
};

} // namespace

TEST(Shape, BaseBookkeepingAccessors)
{
  ShapeHarness shape;

  EXPECT_TRUE(shape.getBoundingBox().getMin().isApprox(
      Eigen::Vector3d(-1.0, -2.0, -3.0)));
  EXPECT_DOUBLE_EQ(shape.getVolume(), 2.0);
  EXPECT_TRUE(shape.computeInertiaFromMass(3.0).isApprox(
      Eigen::Matrix3d::Identity() * 3.0));
  EXPECT_TRUE(shape.computeInertiaFromDensity(4.0).isApprox(
      Eigen::Matrix3d::Identity() * 8.0));
  EXPECT_GT(shape.getID(), 0u);

  shape.setDataVariance(Shape::STATIC);
  EXPECT_EQ(shape.getDataVariance(), Shape::STATIC);
  EXPECT_TRUE(shape.checkDataVariance(Shape::STATIC));
  shape.addDataVariance(Shape::DYNAMIC_COLOR);
  EXPECT_TRUE(shape.checkDataVariance(Shape::DYNAMIC_COLOR));
  shape.removeDataVariance(Shape::DYNAMIC_COLOR);
  EXPECT_FALSE(shape.checkDataVariance(Shape::DYNAMIC_COLOR));

  shape.refreshData();
  shape.notifyAlphaUpdated(0.25);
  shape.notifyColorUpdated(Eigen::Vector4d::Ones());
  EXPECT_NE(shape.clone(), nullptr);
}

TEST(PointCloudShape, PointsAndBoundingBox)
{
  ExposedPointCloudShape shape(0.25);
  EXPECT_DOUBLE_EQ(shape.getVisualSize(), 0.25);
  EXPECT_EQ(shape.getType(), PointCloudShape::getStaticType());
  EXPECT_TRUE(shape.computeInertia(1.0).isApprox(Eigen::Matrix3d::Identity()));

  shape.reserve(3);
  shape.addPoint(Eigen::Vector3d(1.0, 0.0, -1.0));
  std::array<Eigen::Vector3d, 2> extra
      = {Eigen::Vector3d(-2.0, 3.0, 0.5), Eigen::Vector3d(0.5, -1.5, 2.0)};
  shape.addPoint(extra);

  EXPECT_EQ(shape.getNumPoints(), 3u);
  const auto points = shape.getPoints();
  EXPECT_EQ(points.size(), 3u);
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d(1.0, 0.0, -1.0)));

  shape.updateBoundingBox();
  const auto bbox = shape.getBoundingBox();
  EXPECT_VECTOR_NEAR(bbox.getMin(), Eigen::Vector3d(-2.0, -1.5, -1.0), 1e-12);
  EXPECT_VECTOR_NEAR(bbox.getMax(), Eigen::Vector3d(1.0, 3.0, 2.0), 1e-12);

  auto replacement = std::to_array({Eigen::Vector3d(4.0, 5.0, 6.0)});
  shape.setPoint(replacement);
  EXPECT_EQ(shape.getNumPoints(), 1u);
  EXPECT_TRUE(shape.getPoints()[0].isApprox(replacement[0]));

  shape.removeAllPoints();
  shape.updateVolume();
  EXPECT_DOUBLE_EQ(shape.getVolume(), 0.0);
  shape.updateBoundingBox();
  EXPECT_EQ(shape.getNumPoints(), 0u);
  const auto emptyBox = shape.getBoundingBox();
  EXPECT_VECTOR_NEAR(emptyBox.getMin(), Eigen::Vector3d::Zero(), 1e-12);
  EXPECT_VECTOR_NEAR(emptyBox.getMax(), Eigen::Vector3d::Zero(), 1e-12);
}

TEST(PointCloudShape, ColorsAndClone)
{
  PointCloudShape shape(0.1);
  EXPECT_EQ(shape.getPointShapeType(), PointCloudShape::BOX);

  shape.setPointShapeType(PointCloudShape::POINT);
  EXPECT_EQ(shape.getPointShapeType(), PointCloudShape::POINT);

  shape.setColorMode(PointCloudShape::BIND_PER_POINT);
  EXPECT_EQ(shape.getColorMode(), PointCloudShape::BIND_PER_POINT);

  const Eigen::Vector4d defaultColor(0.5, 0.5, 0.5, 0.5);
  EXPECT_VECTOR_NEAR(shape.getOverallColor(), defaultColor, 1e-12);

  const Eigen::Vector4d overall(0.2, 0.3, 0.4, 0.5);
  shape.setOverallColor(overall);
  EXPECT_VECTOR_NEAR(shape.getOverallColor(), overall, 1e-12);

  auto colors = std::to_array(
      {Eigen::Vector4d(0.8, 0.1, 0.1, 1.0),
       Eigen::Vector4d(0.2, 0.9, 0.2, 1.0)});
  shape.setColors(colors);
  const auto colorSpan = shape.getColors();
  EXPECT_EQ(colorSpan.size(), 2u);
  EXPECT_VECTOR_NEAR(colorSpan[0], colors[0], 1e-12);
  EXPECT_VECTOR_NEAR(shape.getOverallColor(), colors[0], 1e-12);

  shape.setVisualSize(0.45);
  EXPECT_DOUBLE_EQ(shape.getVisualSize(), 0.45);

  shape.notifyColorUpdated(colors[0]);

  auto clone = std::dynamic_pointer_cast<PointCloudShape>(shape.clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getPointShapeType(), shape.getPointShapeType());
  EXPECT_EQ(clone->getColorMode(), shape.getColorMode());
  EXPECT_EQ(clone->getNumPoints(), shape.getNumPoints());
  EXPECT_EQ(clone->getColors().size(), shape.getColors().size());
  EXPECT_DOUBLE_EQ(clone->getVisualSize(), shape.getVisualSize());
}

#if DART_HAVE_OCTOMAP
TEST(PointCloudShape, OctomapPointCloudImport)
{
  PointCloudShape shape;

  octomap::Pointcloud pointCloud;
  pointCloud.push_back(1.0, 2.0, 3.0);
  pointCloud.push_back(-1.0, -2.0, -3.0);
  shape.setPoints(pointCloud);

  ASSERT_EQ(shape.getNumPoints(), 2u);
  EXPECT_TRUE(shape.getPoints()[0].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(shape.getPoints()[1].isApprox(Eigen::Vector3d(-1.0, -2.0, -3.0)));

  octomap::Pointcloud extraPoints;
  extraPoints.push_back(0.5, 0.25, 0.125);
  shape.addPoints(extraPoints);

  ASSERT_EQ(shape.getNumPoints(), 3u);
  EXPECT_TRUE(shape.getPoints()[2].isApprox(Eigen::Vector3d(0.5, 0.25, 0.125)));
}
#endif
