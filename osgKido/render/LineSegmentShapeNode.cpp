/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "osgKido/render/LineSegmentShapeNode.hpp"
#include "osgKido/Utils.hpp"

#include "kido/dynamics/LineSegmentShape.hpp"

namespace osgKido {
namespace render {

class LineSegmentShapeGeode : public ShapeNode, public osg::Geode
{
public:

  LineSegmentShapeGeode(std::shared_ptr<kido::dynamics::LineSegmentShape> shape,
                        EntityNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:

  virtual ~LineSegmentShapeGeode();

  std::shared_ptr<kido::dynamics::LineSegmentShape> mLineSegmentShape;
  LineSegmentShapeDrawable* mDrawable;

  osg::ref_ptr<osg::LineWidth> mLineWidth;

};

//==============================================================================
class LineSegmentShapeDrawable : public osg::Geometry
{
public:

  LineSegmentShapeDrawable(kido::dynamics::LineSegmentShape* shape);

  void refresh(bool firstTime);

protected:

  virtual ~LineSegmentShapeDrawable();

  kido::dynamics::LineSegmentShape* mLineSegmentShape;

  osg::ref_ptr<osg::Vec3Array> mVertices;
  osg::ref_ptr<osg::Vec4Array> mColors;
};

//==============================================================================
LineSegmentShapeNode::LineSegmentShapeNode(
    std::shared_ptr<kido::dynamics::LineSegmentShape> shape,
    EntityNode* parent)
  : ShapeNode(shape, parent, this),
    mLineSegmentShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void LineSegmentShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void LineSegmentShapeNode::extractData(bool firstTime)
{
  if(mShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_TRANSFORM)
     || firstTime)
    setMatrix(eigToOsgMatrix(mShape->getLocalTransform()));

  if(nullptr == mGeode)
  {
    mGeode = new LineSegmentShapeGeode(mLineSegmentShape, mParentEntity);
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
    std::shared_ptr<kido::dynamics::LineSegmentShape> shape, EntityNode* parent)
  : ShapeNode(shape, parent, this),
    mLineSegmentShape(shape),
    mDrawable(nullptr),
    mLineWidth(new osg::LineWidth)
{
  getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
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
  if(mLineSegmentShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    mLineWidth->setWidth(mLineSegmentShape->getThickness());
    getOrCreateStateSet()->setAttributeAndModes(mLineWidth);
  }

  if(nullptr == mDrawable)
  {
    mDrawable = new LineSegmentShapeDrawable(mLineSegmentShape.get());
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
    kido::dynamics::LineSegmentShape* shape)
  : mLineSegmentShape(shape),
    mVertices(new osg::Vec3Array),
    mColors(new osg::Vec4Array)
{
  refresh(true);
}

//==============================================================================
void LineSegmentShapeDrawable::refresh(bool firstTime)
{
  if(mLineSegmentShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    setDataVariance(osg::Object::STATIC);
  else
    setDataVariance(osg::Object::DYNAMIC);

  const std::vector<Eigen::Vector3d>& vertices =
      mLineSegmentShape->getVertices();

  const Eigen::aligned_vector<Eigen::Vector2i>& connections =
      mLineSegmentShape->getConnections();

  if(   mLineSegmentShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    osg::ref_ptr<osg::DrawElementsUInt> elements =
        new osg::DrawElementsUInt(GL_LINES);
    elements->reserve(2*connections.size());

    for(size_t i=0; i < connections.size(); ++i)
    {
      const Eigen::Vector2i& c = connections[i];
      elements->push_back(c[0]);
      elements->push_back(c[1]);
    }

    addPrimitiveSet(elements);
  }

  if(   mLineSegmentShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_VERTICES)
     || mLineSegmentShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    if(mVertices->size() != vertices.size())
      mVertices->resize(vertices.size());

    for(size_t i=0; i<vertices.size(); ++i)
      (*mVertices)[i] = eigToOsgVec3(vertices[i]);

    setVertexArray(mVertices);
  }

  if(   mLineSegmentShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    if(mColors->size() != 1)
      mColors->resize(1);

    (*mColors)[0] = eigToOsgVec4(mLineSegmentShape->getRGBA());

    setColorArray(mColors, osg::Array::BIND_OVERALL);
  }
}

//==============================================================================
LineSegmentShapeDrawable::~LineSegmentShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osgKido
