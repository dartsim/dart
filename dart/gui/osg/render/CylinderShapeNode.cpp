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
#include <osg/ShapeDrawable>

#include "dart/gui/osg/render/CylinderShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

class CylinderShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  CylinderShapeGeode(dart::dynamics::CylinderShape* shape,
                     ShapeFrameNode* parent,
                     CylinderShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~CylinderShapeGeode();

  dart::dynamics::CylinderShape* mCylinderShape;
  CylinderShapeDrawable* mDrawable;

};

//==============================================================================
class CylinderShapeDrawable : public ::osg::ShapeDrawable
{
public:

  CylinderShapeDrawable(dart::dynamics::CylinderShape* shape,
                        dart::dynamics::VisualAspect* visualAspect,
                        CylinderShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~CylinderShapeDrawable();

  dart::dynamics::CylinderShape* mCylinderShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  CylinderShapeGeode* mParent;

};

//==============================================================================
CylinderShapeNode::CylinderShapeNode(
    std::shared_ptr<dart::dynamics::CylinderShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mCylinderShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void CylinderShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void CylinderShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new CylinderShapeGeode(mCylinderShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
CylinderShapeNode::~CylinderShapeNode()
{
  // Do nothing
}

//==============================================================================
CylinderShapeGeode::CylinderShapeGeode(
    dart::dynamics::CylinderShape* shape,
    ShapeFrameNode* parent,
    CylinderShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parent, this),
    mCylinderShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void CylinderShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void CylinderShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new CylinderShapeDrawable(mCylinderShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
CylinderShapeGeode::~CylinderShapeGeode()
{
  // Do nothing
}

//==============================================================================
CylinderShapeDrawable::CylinderShapeDrawable(
    dart::dynamics::CylinderShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    CylinderShapeGeode* parent)
  : mCylinderShape(shape),
    mVisualAspect(visualAspect),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void CylinderShapeDrawable::refresh(bool firstTime)
{
  if(mCylinderShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if(mCylinderShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    double R = mCylinderShape->getRadius();
    double h = mCylinderShape->getHeight();
    ::osg::ref_ptr<::osg::Cylinder> osg_shape =
        new ::osg::Cylinder(::osg::Vec3(0,0,0), R, h);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mCylinderShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAspect->getRGBA()));
  }
}

//==============================================================================
CylinderShapeDrawable::~CylinderShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
