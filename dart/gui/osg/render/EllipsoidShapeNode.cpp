/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/gui/osg/render/EllipsoidShapeNode.hpp"

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Light>
#include <osg/Material>
#include <osg/ShapeDrawable>

#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/Utils.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

class EllipsoidShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  EllipsoidShapeGeode(
      dart::dynamics::EllipsoidShape* shape,
      ShapeFrameNode* parentShapeFrame,
      EllipsoidShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~EllipsoidShapeGeode();

  EllipsoidShapeNode* mParentNode;
  dart::dynamics::EllipsoidShape* mEllipsoidShape;
  EllipsoidShapeDrawable* mDrawable;
};

//==============================================================================
class EllipsoidShapeDrawable : public ::osg::ShapeDrawable
{
public:
  EllipsoidShapeDrawable(
      dart::dynamics::EllipsoidShape* shape,
      dart::dynamics::VisualAspect* visualAspect,
      EllipsoidShapeGeode* parent);

  void refresh(bool firstTime);

protected:
  virtual ~EllipsoidShapeDrawable();

  dart::dynamics::EllipsoidShape* mEllipsoidShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  EllipsoidShapeGeode* mParent;
};

//==============================================================================
EllipsoidShapeNode::EllipsoidShapeNode(
    std::shared_ptr<dart::dynamics::EllipsoidShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mEllipsoidShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void EllipsoidShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
double smallestComponent(const Eigen::Vector3d& v)
{
  return std::min(v[0], std::min(v[1], v[2]));
}

//==============================================================================
void EllipsoidShapeNode::extractData(bool firstTime)
{
  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
      || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime)
  {
    Eigen::Matrix4d S(Eigen::Matrix4d::Zero());
    const Eigen::Vector3d& s
        = mEllipsoidShape->getDiameters()
          / smallestComponent(mEllipsoidShape->getDiameters());
    S(0, 0) = s[0];
    S(1, 1) = s[1];
    S(2, 2) = s[2];
    S(3, 3) = 1.0;
    setMatrix(eigToOsgMatrix(S));
  }

  if (nullptr == mGeode)
  {
    mGeode = new EllipsoidShapeGeode(
        mEllipsoidShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
EllipsoidShapeNode::~EllipsoidShapeNode()
{
  // Do nothing
}

//==============================================================================
EllipsoidShapeGeode::EllipsoidShapeGeode(
    dart::dynamics::EllipsoidShape* shape,
    ShapeFrameNode* parentShapeFrame,
    EllipsoidShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mEllipsoidShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  extractData();
}

//==============================================================================
void EllipsoidShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void EllipsoidShapeGeode::extractData()
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new EllipsoidShapeDrawable(mEllipsoidShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
EllipsoidShapeGeode::~EllipsoidShapeGeode()
{
  // Do nothing
}

//==============================================================================
EllipsoidShapeDrawable::EllipsoidShapeDrawable(
    dart::dynamics::EllipsoidShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    EllipsoidShapeGeode* parent)
  : mEllipsoidShape(shape), mVisualAspect(visualAspect), mParent(parent)
{
  refresh(true);
}

//==============================================================================
void EllipsoidShapeDrawable::refresh(bool firstTime)
{
  if (mEllipsoidShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mEllipsoidShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime)
  {
    ::osg::ref_ptr<::osg::Sphere> osg_shape = nullptr;
    osg_shape = new ::osg::Sphere(
        ::osg::Vec3(0, 0, 0), smallestComponent(mEllipsoidShape->getRadii()));

    setShape(osg_shape);
    dirtyDisplayList();
  }

  if (mEllipsoidShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime)
  {
    // Set color
    const ::osg::Vec4d color = eigToOsgVec4d(mVisualAspect->getRGBA());
    setColor(color);

    // Set alpha specific properties
    ::osg::StateSet* ss = getOrCreateStateSet();
    if (std::abs(color.a()) > 1 - getAlphaThreshold())
    {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
    else
    {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
      ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(false);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
  }
}

//==============================================================================
EllipsoidShapeDrawable::~EllipsoidShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
