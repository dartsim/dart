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

#include "dart/gui/osg/render/CapsuleShapeNode.hpp"

#include "dart/dynamics/CapsuleShape.hpp"
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

//==============================================================================
class CapsuleShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  CapsuleShapeGeode(
      dart::dynamics::CapsuleShape* shape,
      ShapeFrameNode* parent,
      CapsuleShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~CapsuleShapeGeode();

  dart::dynamics::CapsuleShape* mCapsuleShape;
  CapsuleShapeDrawable* mDrawable;
};

//==============================================================================
class CapsuleShapeDrawable : public ::osg::ShapeDrawable
{
public:
  CapsuleShapeDrawable(
      dart::dynamics::CapsuleShape* shape,
      dart::dynamics::VisualAspect* visualAspect,
      CapsuleShapeGeode* parent);

  void refresh(bool firstTime);

protected:
  virtual ~CapsuleShapeDrawable();

  dart::dynamics::CapsuleShape* mCapsuleShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  CapsuleShapeGeode* mParent;
};

//==============================================================================
CapsuleShapeNode::CapsuleShapeNode(
    std::shared_ptr<dart::dynamics::CapsuleShape> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mCapsuleShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void CapsuleShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void CapsuleShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode) {
    mGeode = new CapsuleShapeGeode(
        mCapsuleShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
CapsuleShapeNode::~CapsuleShapeNode()
{
  // Do nothing
}

//==============================================================================
CapsuleShapeGeode::CapsuleShapeGeode(
    dart::dynamics::CapsuleShape* shape,
    ShapeFrameNode* parent,
    CapsuleShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parent, this),
    mCapsuleShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  extractData();
}

//==============================================================================
void CapsuleShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void CapsuleShapeGeode::extractData()
{
  if (nullptr == mDrawable) {
    mDrawable = new CapsuleShapeDrawable(mCapsuleShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
CapsuleShapeGeode::~CapsuleShapeGeode()
{
  // Do nothing
}

//==============================================================================
CapsuleShapeDrawable::CapsuleShapeDrawable(
    dart::dynamics::CapsuleShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    CapsuleShapeGeode* parent)
  : mCapsuleShape(shape), mVisualAspect(visualAspect), mParent(parent)
{
  refresh(true);
}

//==============================================================================
void CapsuleShapeDrawable::refresh(bool firstTime)
{
  if (mCapsuleShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mCapsuleShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime) {
    double R = mCapsuleShape->getRadius();
    double h = mCapsuleShape->getHeight();
    ::osg::ref_ptr<::osg::Capsule> osg_shape
        = new ::osg::Capsule(::osg::Vec3(0, 0, 0), R, h);
    setShape(osg_shape);
    dirtyDisplayList();
  }

  if (mCapsuleShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
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
CapsuleShapeDrawable::~CapsuleShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
