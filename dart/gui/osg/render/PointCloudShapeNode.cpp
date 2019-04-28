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

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>

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
class PointCloudShapeDrawable : public ::osg::ShapeDrawable
{
public:
  PointCloudShapeDrawable(
      dart::dynamics::PointCloudShape* shape,
      dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:
  virtual ~PointCloudShapeDrawable();

  dart::dynamics::PointCloudShape* mPointCloudShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;

  ::osg::ref_ptr<::osg::CompositeShape> mOsgShape;

private:
  void addBoxes(
      ::osg::CompositeShape* osgShape,
      const std::vector<Eigen::Vector3d>& points,
      double size);
  std::vector<::osg::ref_ptr<::osg::Box>> mBoxes;
};

//==============================================================================
class PointCloudShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  PointCloudShapeGeode(
      std::shared_ptr<dart::dynamics::PointCloudShape> shape,
      ShapeFrameNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:
  virtual ~PointCloudShapeGeode();

  std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape;
  PointCloudShapeDrawable* mDrawable;
};

//==============================================================================
PointCloudShapeNode::PointCloudShapeNode(
    std::shared_ptr<dart::dynamics::PointCloudShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mPointCloudShape(shape),
    mGeode(nullptr),
    mPointCloudVersion(dynamics::INVALID_INDEX)
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
void PointCloudShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new PointCloudShapeGeode(mPointCloudShape, mParentShapeFrameNode);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
PointCloudShapeNode::~PointCloudShapeNode()
{
  // Do nothing
}

//==============================================================================
PointCloudShapeGeode::PointCloudShapeGeode(
    std::shared_ptr<dart::dynamics::PointCloudShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mPointCloudShape(shape), mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData(true);
}

//==============================================================================
void PointCloudShapeGeode::refresh()
{
  mUtilized = true;

  extractData(false);
}

//==============================================================================
void PointCloudShapeGeode::extractData(bool /*firstTime*/)
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new PointCloudShapeDrawable(mPointCloudShape.get(), mVisualAspect);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
PointCloudShapeGeode::~PointCloudShapeGeode()
{
  // Do nothing
}

//==============================================================================
PointCloudShapeDrawable::PointCloudShapeDrawable(
    dart::dynamics::PointCloudShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mPointCloudShape(shape),
    mVisualAspect(visualAspect),
    mVertices(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array),
    mOsgShape(new ::osg::CompositeShape())
{
  refresh(true);
}

//==============================================================================
void PointCloudShapeDrawable::refresh(bool /*firstTime*/)
{
  if (mPointCloudShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  // This function is called whenever the point cloud version is increased, and
  // the point cloud could be updated in the version up. So we always update the
  // point cloud.
  {
    const auto size = mPointCloudShape->getVisualSize();
    const auto& points = mPointCloudShape->getPoints();

    auto osgShape = new ::osg::CompositeShape();
    addBoxes(osgShape, points, size);

    setShape(osgShape);
    dirtyDisplayList();
  }

  // This function is called whenever the point cloud version is increased, and
  // the color could be updated in the version up. So we always update the
  // color.
  {
    setColor(eigToOsgVec4d(mVisualAspect->getRGBA()));
  }
}

//==============================================================================
void PointCloudShapeDrawable::addBoxes(
    ::osg::CompositeShape* osgShape,
    const std::vector<Eigen::Vector3d>& points,
    double size)
{
  // Pre-allocate for the case that the size of new points are greater than
  // previous update
  mBoxes.reserve(points.size());

  // Update position of cache boxes. The number of being updated boxes is
  // whichever the lower number of cache boxes and new points.
  const auto numUpdatingBoxes = std::min(mBoxes.size(), points.size());
  for (auto i = 0u; i < numUpdatingBoxes; ++i)
  {
    mBoxes[i]->setCenter(eigToOsgVec3(points[i]));
    osgShape->addChild(mBoxes[i]);
  }

  // If the number of new points is greater than cache box, then create new
  // boxes that many.
  for (auto i = mBoxes.size(); i < points.size(); ++i)
  {
    auto osgSphere
        = new ::osg::Box(eigToOsgVec3(points[i]), static_cast<float>(size));
    mBoxes.emplace_back(osgSphere);
    osgShape->addChild(mBoxes.back());
  }

  // Fit the size of cache box list to the new points. No effect new boxes are
  // added to the list.
  mBoxes.resize(points.size());
}

//==============================================================================
PointCloudShapeDrawable::~PointCloudShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
