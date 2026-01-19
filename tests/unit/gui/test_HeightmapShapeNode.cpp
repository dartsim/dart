// Copyright (c) 2011, The DART development contributors

#include <dart/gui/render/detail/heightmap_shape_geometry.hpp>

#include <gtest/gtest.h>

namespace {

using Scalar = float;
using HeightmapShape = dart::dynamics::HeightmapShape<Scalar>;
using HeightField = HeightmapShape::HeightField;
using Vector3 = HeightmapShape::Vector3;

//------------------------------------------------------------------------------
TEST(HeightmapShapeNode, VerticesAreCenteredAroundOrigin)
{
  HeightField heights(2, 3);
  heights.setZero();

  Vector3 scale;
  scale << 2.0f, 3.0f, 1.0f;

  const auto origin
      = dart::gui::render::detail::computeHeightmapVertexOrigin<Scalar>(
          heights, scale);
  const auto p00
      = dart::gui::render::detail::computeHeightmapVertexPosition<Scalar>(
          heights, scale, origin, 0, 0);
  const auto p01
      = dart::gui::render::detail::computeHeightmapVertexPosition<Scalar>(
          heights, scale, origin, 0, 1);
  const auto p10
      = dart::gui::render::detail::computeHeightmapVertexPosition<Scalar>(
          heights, scale, origin, 1, 0);
  const auto p12
      = dart::gui::render::detail::computeHeightmapVertexPosition<Scalar>(
          heights, scale, origin, 1, 2);

  const Scalar spanX
      = (heights.cols() > 1 ? heights.cols() - 1 : 0) * scale.x();
  const Scalar spanY
      = (heights.rows() > 1 ? heights.rows() - 1 : 0) * scale.y();

  EXPECT_FLOAT_EQ(p00.x(), -0.5f * spanX);
  EXPECT_FLOAT_EQ(p00.y(), 0.5f * spanY);
  EXPECT_FLOAT_EQ(p01.x(), 0.0f);
  EXPECT_FLOAT_EQ(p10.y(), -0.5f * spanY);
  EXPECT_FLOAT_EQ(p12.x(), 0.5f * spanX);
  EXPECT_FLOAT_EQ(p12.y(), -0.5f * spanY);
}

} // namespace
