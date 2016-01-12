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
#include <osg/ShapeDrawable>

#include "osgKido/render/CylinderShapeNode.h"
#include "osgKido/Utils.h"

#include "kido/dynamics/CylinderShape.h"

namespace osgKido {
namespace render {

class CylinderShapeGeode : public ShapeNode, public osg::Geode
{
public:

  CylinderShapeGeode(kido::dynamics::CylinderShape* shape,
                     EntityNode* parentEntity,
                     CylinderShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~CylinderShapeGeode();

  kido::dynamics::CylinderShape* mCylinderShape;
  CylinderShapeDrawable* mDrawable;

};

//==============================================================================
class CylinderShapeDrawable : public osg::ShapeDrawable
{
public:

  CylinderShapeDrawable(kido::dynamics::CylinderShape* shape,
                        CylinderShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~CylinderShapeDrawable();

  kido::dynamics::CylinderShape* mCylinderShape;
  CylinderShapeGeode* mParent;

};

//==============================================================================
CylinderShapeNode::CylinderShapeNode(
    std::shared_ptr<kido::dynamics::CylinderShape> shape,
    EntityNode* parent)
  : ShapeNode(shape, parent, this),
    mCylinderShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void CylinderShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void CylinderShapeNode::extractData(bool firstTime)
{
  if(mShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_TRANSFORM)
     || firstTime)
    setMatrix(eigToOsgMatrix(mShape->getLocalTransform()));

  if(nullptr == mGeode)
  {
    mGeode = new CylinderShapeGeode(mCylinderShape.get(), mParentEntity, this);
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
CylinderShapeGeode::CylinderShapeGeode(kido::dynamics::CylinderShape* shape,
    EntityNode* parentEntity,
    CylinderShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentEntity, this),
    mCylinderShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
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
    mDrawable = new CylinderShapeDrawable(mCylinderShape, this);
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
    kido::dynamics::CylinderShape* shape, CylinderShapeGeode* parent)
  : mCylinderShape(shape),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void CylinderShapeDrawable::refresh(bool firstTime)
{
  if(mCylinderShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    setDataVariance(osg::Object::STATIC);
  else
    setDataVariance(osg::Object::DYNAMIC);

  if(mCylinderShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    double R = mCylinderShape->getRadius();
    double h = mCylinderShape->getHeight();
    osg::ref_ptr<osg::Cylinder> osg_shape =
        new osg::Cylinder(osg::Vec3(0,0,0), R, h);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mCylinderShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mCylinderShape->getRGBA()));
  }
}

//==============================================================================
CylinderShapeDrawable::~CylinderShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osgKido
