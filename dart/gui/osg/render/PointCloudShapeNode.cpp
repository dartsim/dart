/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <osg/Billboard>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>

#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"
#include "dart/gui/osg/render/PointCloudShapeNode.hpp"

#include "dart/dynamics/PointCloudShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class BoxDrawable final : public ::osg::ShapeDrawable
{
public:
  BoxDrawable(double size, const Eigen::Vector4d& color)
  {
    mShape = new ::osg::Box(::osg::Vec3(), static_cast<float>(size));
    setColor(eigToOsgVec4f(color));
    setShape(mShape);
    setDataVariance(::osg::Object::DYNAMIC);
  }

  void updateSize(double size)
  {
    mShape->setHalfLengths(::osg::Vec3(
        static_cast<float>(size * 0.5),
        static_cast<float>(size * 0.5),
        static_cast<float>(size * 0.5)));
    dirtyBound();
    dirtyDisplayList();
  }

  void updateColor(const Eigen::Vector4d& color)
  {
    setColor(eigToOsgVec4f(color));
  }

protected:
  ::osg::ref_ptr<::osg::Box> mShape;
};

//==============================================================================
class QuadDrawable : public ::osg::Geometry
{
public:
  QuadDrawable(double size, const Eigen::Vector4d& color)
  {
    mVertices = new ::osg::Vec3Array(4);
    mNormals = new ::osg::Vec3Array(1);
    mColors = new ::osg::Vec4Array(1);

    updateSize(size);
    updateColor(color);

    addPrimitiveSet(new ::osg::DrawArrays(::osg::PrimitiveSet::QUADS, 0, 4));

    setDataVariance(::osg::Object::DYNAMIC);
  }

  void updateSize(double size)
  {
    mHalfSize = static_cast<float>(size) / 2.0f;

    mVertices->at(0) = ::osg::Vec3(-mHalfSize, 0, -mHalfSize);
    mVertices->at(1) = ::osg::Vec3(+mHalfSize, 0, -mHalfSize);
    mVertices->at(2) = ::osg::Vec3(+mHalfSize, 0, +mHalfSize);
    mVertices->at(3) = ::osg::Vec3(-mHalfSize, 0, +mHalfSize);
    setVertexArray(mVertices);

    mNormals->at(0) = ::osg::Vec3(0, -1, 0);
    setNormalArray(mNormals, ::osg::Array::BIND_OVERALL);
  }

  void updateColor(const Eigen::Vector4d& color)
  {
    mColors->at(0) = eigToOsgVec4f(color);
    setColorArray(mColors);
    setColorBinding(::osg::Geometry::BIND_OVERALL);
  }

protected:
  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
  float mHalfSize;
};

//==============================================================================
class CircleDrawable : public ::osg::Geometry
{
public:
  CircleDrawable(double size, const Eigen::Vector4d& color)
  {
    mVertices = new ::osg::Vec3Array(4);
    mNormals = new ::osg::Vec3Array(1);
    mColors = new ::osg::Vec4Array(1);

    addPrimitiveSet(new ::osg::DrawArrays(::osg::PrimitiveSet::POLYGON, 0, 0));

    updateSize(size);
    updateColor(color);

    setDataVariance(::osg::Object::DYNAMIC);
  }

  void updateSize(double size)
  {
    mRadius = static_cast<float>(size) / 2.0f;

    const auto segmentCount = 16u;
    const auto pi = math::constantsf::pi();
    const auto ratio = 2.0f * pi / segmentCount;

    mVertices->resize(segmentCount);
    for (auto i = 0u; i < segmentCount; ++i)
    {
      const auto angle = i * ratio;
      mVertices->at(i) = ::osg::Vec3(
          mRadius * ::std::cos(angle), 0, mRadius * ::std::sin(angle));
    }
    setVertexArray(mVertices);

    mNormals->at(0) = ::osg::Vec3(0, -1, 0);
    setNormalArray(mNormals, ::osg::Array::BIND_OVERALL);

    setPrimitiveSet(
        0,
        new ::osg::DrawArrays(::osg::PrimitiveSet::POLYGON, 0, segmentCount));
  }

  void updateColor(const Eigen::Vector4d& color)
  {
    mColors->at(0) = eigToOsgVec4f(color);
    setColorArray(mColors);
    setColorBinding(::osg::Geometry::BIND_OVERALL);
  }

protected:
  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
  float mRadius;
};

//==============================================================================
class PointNode : public ::osg::MatrixTransform
{
public:
  PointNode() = default;

  void updateCenter(const Eigen::Vector3d& point)
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = point;
    setMatrix(eigToOsgMatrix(tf));
  }

  virtual void updateSize(double size) = 0;
  virtual void updateColor(const Eigen::Vector4d& color) = 0;
};

//==============================================================================
class BoxPointNode final : public PointNode
{
public:
  BoxPointNode(
      const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
  {
    mDrawable = new BoxDrawable(size, color);
    mGeode = new ::osg::Geode();

    mGeode->addDrawable(mDrawable);
    addChild(mGeode);

    updateCenter(point);
  }

  void updateSize(double size) override
  {
    mDrawable->updateSize(size);
  }

  void updateColor(const Eigen::Vector4d& color) override
  {
    mDrawable->updateColor(color);
  }

protected:
  ::osg::ref_ptr<BoxDrawable> mDrawable;
  ::osg::ref_ptr<::osg::Geode> mGeode;
};

//==============================================================================
template <typename T>
class BillboardPointNode final : public PointNode
{
public:
  BillboardPointNode(
      const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
  {
    mDrawable = new T(size, color);
    mGeode = new ::osg::Billboard();
    mGeode->setMode(::osg::Billboard::POINT_ROT_EYE);

    mGeode->addDrawable(mDrawable);
    addChild(mGeode);

    updateCenter(point);
  }

  void updateSize(double size) override
  {
    mDrawable->updateSize(size);
  }

  void updateColor(const Eigen::Vector4d& color) override
  {
    mDrawable->updateColor(color);
  }

protected:
  ::osg::ref_ptr<T> mDrawable;
  ::osg::ref_ptr<::osg::Billboard> mGeode;
};

//==============================================================================
PointCloudShapeNode::PointCloudShapeNode(
    std::shared_ptr<dart::dynamics::PointCloudShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mPointCloudShape(shape),
    mPointCloudVersion(dynamics::INVALID_INDEX),
    mPointShapeType(shape->getPointShapeType())
{
  mNode = this;
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);
}

//==============================================================================
void PointCloudShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC
      && mPointCloudVersion == mPointCloudShape->getVersion())
  {
    return;
  }

  extractData(false);

  mPointCloudVersion = mPointCloudShape->getVersion();
}

//==============================================================================
bool shouldUseVisualAspectColor(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<
        Eigen::Vector4d,
        Eigen::aligned_allocator<Eigen::Vector4d>>& colors,
    dynamics::PointCloudShape::ColorMode colorMode)
{
  if (colorMode == dynamics::PointCloudShape::USE_SHAPE_COLOR)
  {
    return true;
  }
  else if (colorMode == dynamics::PointCloudShape::BIND_OVERALL)
  {
    if (colors.empty())
    {
      dtwarn << "[PointCloudShapeNode] The color array in PointCloudShape "
             << "is empty while the color mode is BIND_OVERALL, which "
             << "requires at least on color in the color array. "
             << "Using visual aspect color instead.\n";
      return true;
    }
  }
  else if (colorMode == dynamics::PointCloudShape::BIND_PER_POINT)
  {
    if (colors.size() != points.size())
    {
      dtwarn << "[PointCloudShapeNode] The color array in PointCloudShape "
             << "has different size from the point array while the color mode "
             << "is BIND_PER_POINT, which requires the same number of colors. "
             << "Using visual aspect color instead.\n";
      return true;
    }
  }
  else
  {
    dtwarn << "[PointCloudShapeNode] The current color mode '" << colorMode
           << "' is not supported by OSG renderer.\n";
    return true;
  }

  return false;
}

//==============================================================================
void PointCloudShapeNode::extractData(bool firstTime)
{
  if (firstTime)
  {
    mPointShapeType = mPointCloudShape->getPointShapeType();
  }
  else if (mPointShapeType != mPointCloudShape->getPointShapeType())
  {
    mPointNodes.clear();
    mPointShapeType = mPointCloudShape->getPointShapeType();
    removeChildren(0, getNumChildren());
  }

  const auto visualSize = mPointCloudShape->getVisualSize();
  const auto& points = mPointCloudShape->getPoints();
  const auto& colors = mPointCloudShape->getColors();
  const auto colorMode = mPointCloudShape->getColorMode();

  const bool useVisualAspectColor
      = shouldUseVisualAspectColor(points, colors, colorMode);

  // Pre-allocate for the case that the size of new points are greater than
  // previous update
  mPointNodes.reserve(points.size());

  // Update position of cache boxes. The number of being updated boxes is
  // whichever the lower number of cache boxes and new points.
  const auto numUpdatingPoints = std::min(mPointNodes.size(), points.size());
  for (auto i = 0u; i < numUpdatingPoints; ++i)
  {
    mPointNodes[i]->updateCenter(points[i]);
    mPointNodes[i]->updateSize(visualSize);
    if (useVisualAspectColor
        || colorMode == dynamics::PointCloudShape::USE_SHAPE_COLOR)
    {
      mPointNodes[i]->updateColor(mVisualAspect->getRGBA());
    }
    else if (colorMode == dynamics::PointCloudShape::BIND_OVERALL)
    {
      mPointNodes[i]->updateColor(colors[0]);
    }
    else if (colorMode == dynamics::PointCloudShape::BIND_PER_POINT)
    {
      mPointNodes[i]->updateColor(colors[i]);
    }
  }

  // If the number of new points is greater than cache box, then create new
  // boxes that many.
  for (auto i = mPointNodes.size(); i < points.size(); ++i)
  {
    ::osg::ref_ptr<PointNode> pointNode;
    if (useVisualAspectColor
        || colorMode == dynamics::PointCloudShape::USE_SHAPE_COLOR)
    {
      pointNode
          = createPointNode(points[i], visualSize, mVisualAspect->getRGBA());
    }
    else if (colorMode == dynamics::PointCloudShape::BIND_OVERALL)
    {
      pointNode = createPointNode(points[i], visualSize, colors[0]);
    }
    else if (colorMode == dynamics::PointCloudShape::BIND_PER_POINT)
    {
      pointNode = createPointNode(points[i], visualSize, colors[i]);
    }
    mPointNodes.emplace_back(pointNode);
    addChild(mPointNodes.back());
  }

  // Fit the size of cache box list to the new points. No effect new boxes are
  // added to the list.
  if (mPointNodes.size() > points.size())
  {
    removeChildren(
        static_cast<unsigned int>(points.size()),
        static_cast<unsigned int>(mPointNodes.size() - points.size()));
    mPointNodes.resize(points.size());
  }
}

//==============================================================================
PointCloudShapeNode::~PointCloudShapeNode()
{
  // Do nothing
}

//==============================================================================
::osg::ref_ptr<PointNode> PointCloudShapeNode::createPointNode(
    const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
{
  if (mPointShapeType == dynamics::PointCloudShape::PointShapeType::BOX)
  {
    return new BoxPointNode(point, size, color);
  }
  else if (
      mPointShapeType
      == dynamics::PointCloudShape::PointShapeType::BILLBOARD_QUAD)
  {
    return new BillboardPointNode<QuadDrawable>(point, size, color);
  }
  else if (
      mPointShapeType
      == dynamics::PointCloudShape::PointShapeType::BILLBOARD_CIRCLE)
  {
    return new BillboardPointNode<CircleDrawable>(point, size, color);
  }

  return nullptr;
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
