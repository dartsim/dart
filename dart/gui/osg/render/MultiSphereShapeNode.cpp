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
#include <osg/Light>
#include <osg/Material>

#include "dart/gui/osg/render/MultiSphereShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class MultiSphereShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  MultiSphereShapeGeode(dart::dynamics::MultiSphereShape* shape,
                        ShapeFrameNode* parentShapeFrame,
                        MultiSphereShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~MultiSphereShapeGeode();

  MultiSphereShapeNode* mParentNode;
  dart::dynamics::MultiSphereShape* mMultiSphereShape;
  MultiSphereShapeDrawable* mDrawable;

};

//==============================================================================
class MultiSphereShapeDrawable : public ::osg::ShapeDrawable
{
public:

  MultiSphereShapeDrawable(dart::dynamics::MultiSphereShape* shape,
                           dart::dynamics::VisualAspect* visualAspect,
                           MultiSphereShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~MultiSphereShapeDrawable();

  dart::dynamics::MultiSphereShape* mMultiSphereShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  MultiSphereShapeGeode* mParent;

};

//==============================================================================
MultiSphereShapeNode::MultiSphereShapeNode(
    std::shared_ptr<dart::dynamics::MultiSphereShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mMultiSphereShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void MultiSphereShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MultiSphereShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new MultiSphereShapeGeode(
          mMultiSphereShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
MultiSphereShapeNode::~MultiSphereShapeNode()
{
  // Do nothing
}

//==============================================================================
MultiSphereShapeGeode::MultiSphereShapeGeode(
    dart::dynamics::MultiSphereShape* shape,
    ShapeFrameNode* parentShapeFrame,
    MultiSphereShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mMultiSphereShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void MultiSphereShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void MultiSphereShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new MultiSphereShapeDrawable(
          mMultiSphereShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
MultiSphereShapeGeode::~MultiSphereShapeGeode()
{
  // Do nothing
}

//==============================================================================
MultiSphereShapeDrawable::MultiSphereShapeDrawable(
    dart::dynamics::MultiSphereShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    MultiSphereShapeGeode* parent)
  : mMultiSphereShape(shape),
    mVisualAspect(visualAspect),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void MultiSphereShapeDrawable::refresh(bool firstTime)
{
  if(mMultiSphereShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if(mMultiSphereShape->checkDataVariance(
       dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    ::osg::ref_ptr<::osg::CompositeShape> osg_shape = nullptr;
    osg_shape = new ::osg::CompositeShape();

    const auto& spheres = mMultiSphereShape->getSpheres();
    for (const auto& sphere : spheres)
    {
      ::osg::ref_ptr<::osg::Sphere> osg_sphere = nullptr;
      osg_sphere = new ::osg::Sphere(
          ::osg::Vec3(sphere.second.x(), sphere.second.y(), sphere.second.z()),
          sphere.first);
      osg_shape->addChild(osg_sphere);
    }

    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mMultiSphereShape->checkDataVariance(
       dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAspect->getRGBA()));
  }
}

//==============================================================================
MultiSphereShapeDrawable::~MultiSphereShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
