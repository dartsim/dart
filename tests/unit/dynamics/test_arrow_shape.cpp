// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/arrow_shape.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
TEST(ArrowShapeTest, DefaultConstructor)
{
  ArrowShape arrow;

  EXPECT_EQ(arrow.getType(), MeshShape::getStaticType());
}

//==============================================================================
TEST(ArrowShapeTest, ConstructorWithPositions)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  Eigen::Vector4d color(1.0, 0.0, 0.0, 1.0);

  ArrowShape arrow(tail, head, ArrowShape::Properties(), color);

  EXPECT_TRUE(arrow.getTail().isApprox(tail));
  EXPECT_TRUE(arrow.getHead().isApprox(head));
}

//==============================================================================
TEST(ArrowShapeTest, SetPositions)
{
  ArrowShape arrow;

  Eigen::Vector3d newTail(1.0, 2.0, 3.0);
  Eigen::Vector3d newHead(4.0, 5.0, 6.0);
  arrow.setPositions(newTail, newHead);

  EXPECT_TRUE(arrow.getTail().isApprox(newTail));
  EXPECT_TRUE(arrow.getHead().isApprox(newHead));
}

//==============================================================================
TEST(ArrowShapeTest, Properties)
{
  ArrowShape::Properties props(0.02, 2.0, 0.25, 0.01, 0.1, false);

  EXPECT_DOUBLE_EQ(props.mRadius, 0.02);
  EXPECT_DOUBLE_EQ(props.mHeadRadiusScale, 2.0);
  EXPECT_DOUBLE_EQ(props.mHeadLengthScale, 0.25);
  EXPECT_DOUBLE_EQ(props.mMinHeadLength, 0.01);
  EXPECT_DOUBLE_EQ(props.mMaxHeadLength, 0.1);
  EXPECT_FALSE(props.mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, SetProperties)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  ArrowShape arrow(tail, head);

  ArrowShape::Properties newProps(0.05, 3.0, 0.3, 0.02, 0.2, true);
  arrow.setProperties(newProps);

  const auto& props = arrow.getProperties();
  EXPECT_DOUBLE_EQ(props.mRadius, 0.05);
  EXPECT_DOUBLE_EQ(props.mHeadRadiusScale, 3.0);
  EXPECT_TRUE(props.mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrow)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(2.0, 0.0, 0.0);
  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ArrowShape arrow(tail, head, props);

  EXPECT_TRUE(arrow.getProperties().mDoubleArrow);
}

//==============================================================================
TEST(ArrowShapeTest, Clone)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 2.0, 3.0);
  ArrowShape::Properties props(0.03, 2.5, 0.2, 0.01, 0.15, false);
  Eigen::Vector4d color(0.5, 0.5, 0.5, 1.0);

  auto original = std::make_shared<ArrowShape>(tail, head, props, color);
  auto cloned = original->clone();
  auto clonedArrow = std::dynamic_pointer_cast<ArrowShape>(cloned);

  ASSERT_NE(clonedArrow, nullptr);
  EXPECT_TRUE(clonedArrow->getTail().isApprox(tail));
  EXPECT_TRUE(clonedArrow->getHead().isApprox(head));
  EXPECT_NE(clonedArrow.get(), original.get());
}

//==============================================================================
TEST(ArrowShapeTest, ColorUpdate)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);
  Eigen::Vector4d initialColor(1.0, 0.0, 0.0, 1.0);

  ArrowShape arrow(tail, head, ArrowShape::Properties(), initialColor);

  Eigen::Vector4d newColor(0.0, 1.0, 0.0, 0.5);
  arrow.notifyColorUpdated(newColor);

  EXPECT_FALSE(arrow.getMaterials().empty());
}

//==============================================================================
TEST(ArrowShapeTest, PropertiesClampedToValidRange)
{
  Eigen::Vector3d tail(0.0, 0.0, 0.0);
  Eigen::Vector3d head(1.0, 0.0, 0.0);

  ArrowShape::Properties props;
  props.mHeadLengthScale = 1.5;
  props.mMinHeadLength = -0.1;
  props.mMaxHeadLength = -0.2;
  props.mHeadRadiusScale = 0.5;

  ArrowShape arrow(tail, head, props);

  const auto& clampedProps = arrow.getProperties();
  EXPECT_LE(clampedProps.mHeadLengthScale, 1.0);
  EXPECT_GE(clampedProps.mHeadLengthScale, 0.0);
  EXPECT_GE(clampedProps.mMinHeadLength, 0.0);
  EXPECT_GE(clampedProps.mMaxHeadLength, 0.0);
  EXPECT_GE(clampedProps.mHeadRadiusScale, 1.0);
}

//==============================================================================
TEST(ArrowShapeTest, SingleArrowMeshGeneration)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(0.0, 0.0, 1.0);
  const std::size_t resolution = 6;

  ArrowShape arrow(
      tail,
      head,
      ArrowShape::Properties(),
      Eigen::Vector4d::Ones(),
      resolution);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getVertices().size(), 4 * resolution + 1);
  EXPECT_EQ(mesh->getTriangles().size(), 5 * resolution);
}

//==============================================================================
TEST(ArrowShapeTest, DoubleArrowMeshGeneration)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(0.0, 0.0, 1.0);
  const std::size_t resolution = 6;

  ArrowShape::Properties props;
  props.mDoubleArrow = true;

  ArrowShape arrow(tail, head, props, Eigen::Vector4d::Ones(), resolution);

  const auto mesh = arrow.getTriMesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getVertices().size(), 6 * resolution + 2);
  EXPECT_EQ(mesh->getTriangles().size(), 8 * resolution);
}

//==============================================================================
TEST(ArrowShapeTest, ReconfigureReleasesCachedScene)
{
  const Eigen::Vector3d tail(0.0, 0.0, 0.0);
  const Eigen::Vector3d head(1.0, 0.0, 0.0);
  ArrowShape arrow(tail, head);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const aiScene* scene = arrow.getMesh();
#pragma GCC diagnostic pop

  ASSERT_NE(scene, nullptr);

  const Eigen::Vector3d newTail(0.0, 0.0, 0.0);
  const Eigen::Vector3d newHead(0.0, 1.0, 0.0);
  arrow.configureArrow(newTail, newHead, arrow.getProperties());
}

//==============================================================================
TEST(ArrowShapeTest, AntiparallelDirection)
{
  const Eigen::Vector3d tail(0.0, 0.0, 1.0);
  const Eigen::Vector3d head(0.0, 0.0, 0.0);

  ArrowShape arrow(tail, head);

  EXPECT_NE(arrow.getTriMesh(), nullptr);
}
