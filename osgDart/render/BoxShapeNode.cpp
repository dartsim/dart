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

#include "osgDart/render/BoxShapeNode.h"
#include "osgDart/Utils.h"

#include "dart/dynamics/BoxShape.h"

namespace osgDart {
namespace render {

class BoxShapeGeode : public ShapeNode, public osg::Geode
{
public:

  BoxShapeGeode(kido::dynamics::BoxShape* shape,
                EntityNode* parentEntity,
                BoxShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~BoxShapeGeode();

  kido::dynamics::BoxShape* mBoxShape;
  BoxShapeDrawable* mDrawable;

};

//==============================================================================
class BoxShapeDrawable : public osg::ShapeDrawable
{
public:

  BoxShapeDrawable(kido::dynamics::BoxShape* shape);

  void refresh(bool firstTime);

protected:

  virtual ~BoxShapeDrawable();

  kido::dynamics::BoxShape* mBoxShape;

};

//==============================================================================
BoxShapeNode::BoxShapeNode(std::shared_ptr<kido::dynamics::BoxShape> shape,
                           EntityNode* parent)
  : ShapeNode(shape, parent, this),
    mBoxShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void BoxShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void BoxShapeNode::extractData(bool firstTime)
{
  if(mShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_TRANSFORM)
     || firstTime)
    setMatrix(eigToOsgMatrix(mShape->getLocalTransform()));

  if(nullptr == mGeode)
  {
    mGeode = new BoxShapeGeode(mBoxShape.get(), mParentEntity, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
BoxShapeNode::~BoxShapeNode()
{
  // Do nothing
}

//==============================================================================
BoxShapeGeode::BoxShapeGeode(kido::dynamics::BoxShape* shape,
                             EntityNode* parent,
                             BoxShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parent, this),
    mBoxShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  extractData();
}

//==============================================================================
void BoxShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void BoxShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new BoxShapeDrawable(mBoxShape);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
BoxShapeGeode::~BoxShapeGeode()
{
  // Do nothing
}

//==============================================================================
BoxShapeDrawable::BoxShapeDrawable(kido::dynamics::BoxShape* shape)
  : mBoxShape(shape)
{
  refresh(true);
}

//==============================================================================
void BoxShapeDrawable::refresh(bool firstTime)
{
  if(mBoxShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    setDataVariance(osg::Object::STATIC);
  else
    setDataVariance(osg::Object::DYNAMIC);

  if(mBoxShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    const Eigen::Vector3d& d = mBoxShape->getSize();
    osg::ref_ptr<osg::Box> osg_shape = new osg::Box(osg::Vec3(0,0,0),
                                                    d[0], d[1], d[2]);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mBoxShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mBoxShape->getRGBA()));
  }
}

//==============================================================================
BoxShapeDrawable::~BoxShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osgDart
