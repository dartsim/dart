/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/gui/osg/render/CylinderShapeNode.hpp"

#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/Utils.hpp"

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/ShapeDrawable>

namespace dart {
namespace gui {
namespace osg {
namespace render {

class CylinderShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  CylinderShapeGeode(
      dart::dynamics::CylinderShape* shape,
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
  CylinderShapeDrawable(
      dart::dynamics::CylinderShape* shape,
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
  : ShapeNode(shape, parent, this), mCylinderShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void CylinderShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void CylinderShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode) {
    mGeode = new CylinderShapeGeode(
        mCylinderShape.get(), mParentShapeFrameNode, this);
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
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
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
  if (nullptr == mDrawable) {
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
  : mCylinderShape(shape), mVisualAspect(visualAspect), mParent(parent)
{
  refresh(true);
}

//==============================================================================
void CylinderShapeDrawable::refresh(bool firstTime)
{
  if (mCylinderShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mCylinderShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime) {
    double R = mCylinderShape->getRadius();
    double h = mCylinderShape->getHeight();
    ::osg::ref_ptr<::osg::Cylinder> osg_shape
        = new ::osg::Cylinder(::osg::Vec3(0, 0, 0), R, h);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if (mCylinderShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime) {
    // Set color
    const ::osg::Vec4d color = eigToOsgVec4d(mVisualAspect->getRGBA());
    setColor(color);

    // Set alpha specific properties
    ::osg::StateSet* ss = getOrCreateStateSet();
    if (std::abs(color.a()) > 1 - getAlphaThreshold()) {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    } else {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
      ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(false);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
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
