/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/gui/osg/render/ConeShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class ConeShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  ConeShapeGeode(dart::dynamics::ConeShape* shape,
                 ShapeFrameNode* parent,
                 ConeShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~ConeShapeGeode();

  dart::dynamics::ConeShape* mConeShape;
  ConeShapeDrawable* mDrawable;

};

//==============================================================================
class ConeShapeDrawable : public ::osg::ShapeDrawable
{
public:

  ConeShapeDrawable(dart::dynamics::ConeShape* shape,
                    dart::dynamics::VisualAspect* visualAspect,
                    ConeShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~ConeShapeDrawable();

  dart::dynamics::ConeShape* mConeShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  ConeShapeGeode* mParent;

};

//==============================================================================
ConeShapeNode::ConeShapeNode(
    std::shared_ptr<dart::dynamics::ConeShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mConeShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void ConeShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void ConeShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new ConeShapeGeode(mConeShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
ConeShapeNode::~ConeShapeNode()
{
  // Do nothing
}

//==============================================================================
ConeShapeGeode::ConeShapeGeode(
    dart::dynamics::ConeShape* shape,
    ShapeFrameNode* parent,
    ConeShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parent, this),
    mConeShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void ConeShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void ConeShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new ConeShapeDrawable(mConeShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
ConeShapeGeode::~ConeShapeGeode()
{
  // Do nothing
}

//==============================================================================
ConeShapeDrawable::ConeShapeDrawable(
    dart::dynamics::ConeShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    ConeShapeGeode* parent)
  : mConeShape(shape),
    mVisualAspect(visualAspect),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void ConeShapeDrawable::refresh(bool firstTime)
{
  if(mConeShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if(mConeShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    double R = mConeShape->getRadius();
    double h = mConeShape->getHeight();
    ::osg::ref_ptr<::osg::Cone> osg_shape =
        new ::osg::Cone(::osg::Vec3(0,0,0), R, h);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mConeShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAspect->getRGBA()));
  }
}

//==============================================================================
ConeShapeDrawable::~ConeShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
