// Copyright (c) 2011-2025, The DART development contributors

#include <dart/gui/osg/render/HeightmapShapeNode.hpp>

#include <dart/dynamics/HeightmapShape.hpp>

#include <gtest/gtest.h>

namespace {

using Scalar = float;
using HeightField = dart::dynamics::HeightmapShape<Scalar>::HeightField;
using Drawable = dart::gui::osg::render::HeightmapShapeDrawable<Scalar>;

//------------------------------------------------------------------------------
TEST(HeightmapShapeNode, VerticesAreCenteredAroundOrigin)
{
  HeightField heights(2, 3);
  heights.setZero();

  ::osg::ref_ptr<Drawable::Vec3Array> vertices = new Drawable::Vec3Array;
  ::osg::ref_ptr<Drawable::Vec3Array> normals = new Drawable::Vec3Array;
  ::osg::ref_ptr<::osg::DrawElementsUInt> faces
      = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::TRIANGLES);

  typename Drawable::Vector3 scale;
  scale << 2.0f, 3.0f, 1.0f;

  dart::gui::osg::render::setVertices<Scalar>(
      heights, *vertices, *faces, *normals, scale);

  ASSERT_EQ(vertices->size(), heights.size());

  const Scalar spanX
      = (heights.cols() > 1 ? heights.cols() - 1 : 0) * scale.x();
  const Scalar spanY
      = (heights.rows() > 1 ? heights.rows() - 1 : 0) * scale.y();

  EXPECT_FLOAT_EQ(vertices->front().x(), -0.5f * spanX);
  EXPECT_FLOAT_EQ(vertices->front().y(), 0.5f * spanY);
  EXPECT_FLOAT_EQ((*vertices)[1].x(), 0.0f);
  EXPECT_FLOAT_EQ((*vertices)[3].y(), -0.5f * spanY);
  EXPECT_FLOAT_EQ(vertices->back().x(), 0.5f * spanX);
  EXPECT_FLOAT_EQ(vertices->back().y(), -0.5f * spanY);
}

} // namespace
