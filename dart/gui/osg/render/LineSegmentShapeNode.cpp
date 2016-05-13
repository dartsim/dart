/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/gui/osg/render/LineSegmentShapeNode.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

class LineSegmentShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  LineSegmentShapeGeode(std::shared_ptr<dart::dynamics::LineSegmentShape> shape,
                        ShapeFrameNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:

  virtual ~LineSegmentShapeGeode();

  std::shared_ptr<dart::dynamics::LineSegmentShape> mLineSegmentShape;
  LineSegmentShapeDrawable* mDrawable;

  ::osg::ref_ptr<::osg::LineWidth> mLineWidth;

};

//==============================================================================
class LineSegmentShapeDrawable : public ::osg::Geometry
{
public:

  LineSegmentShapeDrawable(dart::dynamics::LineSegmentShape* shape,
                           dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:

  virtual ~LineSegmentShapeDrawable();

  dart::dynamics::LineSegmentShape* mLineSegmentShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
};

//==============================================================================
LineSegmentShapeNode::LineSegmentShapeNode(
    std::shared_ptr<dart::dynamics::LineSegmentShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mLineSegmentShape(shape),
    mGeode(nullptr)
{
  mNode = this;
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void LineSegmentShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void LineSegmentShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new LineSegmentShapeGeode(mLineSegmentShape, mParentShapeFrameNode);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
LineSegmentShapeNode::~LineSegmentShapeNode()
{
  // Do nothing
}

//==============================================================================
LineSegmentShapeGeode::LineSegmentShapeGeode(
    std::shared_ptr<dart::dynamics::LineSegmentShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mLineSegmentShape(shape),
    mDrawable(nullptr),
    mLineWidth(new ::osg::LineWidth)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  getOrCreateStateSet()->setMode(GL_LIGHTING, ::osg::StateAttribute::OFF);
  extractData(true);
}

//==============================================================================
void LineSegmentShapeGeode::refresh()
{
  mUtilized = true;

  extractData(false);
}

//==============================================================================
void LineSegmentShapeGeode::extractData(bool firstTime)
{
  if(mLineSegmentShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    mLineWidth->setWidth(mLineSegmentShape->getThickness());
    getOrCreateStateSet()->setAttributeAndModes(mLineWidth);
  }

  if(nullptr == mDrawable)
  {
    mDrawable = new LineSegmentShapeDrawable(mLineSegmentShape.get(), mVisualAspect);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
LineSegmentShapeGeode::~LineSegmentShapeGeode()
{
  // Do nothing
}

//==============================================================================
LineSegmentShapeDrawable::LineSegmentShapeDrawable(
    dart::dynamics::LineSegmentShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mLineSegmentShape(shape),
    mVisualAspect(visualAspect),
    mVertices(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array)
{
  refresh(true);
}

//==============================================================================
void LineSegmentShapeDrawable::refresh(bool firstTime)
{
  if(mLineSegmentShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  const std::vector<Eigen::Vector3d>& vertices =
      mLineSegmentShape->getVertices();

  const Eigen::aligned_vector<Eigen::Vector2i>& connections =
      mLineSegmentShape->getConnections();

  if(   mLineSegmentShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    ::osg::ref_ptr<::osg::DrawElementsUInt> elements =
        new ::osg::DrawElementsUInt(GL_LINES);
    elements->reserve(2*connections.size());

    for(std::size_t i=0; i < connections.size(); ++i)
    {
      const Eigen::Vector2i& c = connections[i];
      elements->push_back(c[0]);
      elements->push_back(c[1]);
    }

    addPrimitiveSet(elements);
  }

  if(   mLineSegmentShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES)
     || mLineSegmentShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    if(mVertices->size() != vertices.size())
      mVertices->resize(vertices.size());

    for(std::size_t i=0; i<vertices.size(); ++i)
      (*mVertices)[i] = eigToOsgVec3(vertices[i]);

    setVertexArray(mVertices);
  }

  if(   mLineSegmentShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    if(mColors->size() != 1)
      mColors->resize(1);

    (*mColors)[0] = eigToOsgVec4(mVisualAspect->getRGBA());

    setColorArray(mColors, ::osg::Array::BIND_OVERALL);
  }
}

//==============================================================================
LineSegmentShapeDrawable::~LineSegmentShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
