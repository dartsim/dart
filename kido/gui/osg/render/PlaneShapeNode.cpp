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

#include "kido/gui/osg/render/PlaneShapeNode.hpp"
#include "kido/gui/osg/Utils.hpp"

#include "kido/dynamics/PlaneShape.hpp"

namespace kido {
namespace gui {
namespace osg {
namespace render {

class PlaneShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  PlaneShapeGeode(kido::dynamics::PlaneShape* shape,
                  EntityNode* parentEntity,
                  PlaneShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~PlaneShapeGeode();

  kido::dynamics::PlaneShape* mPlaneShape;
  PlaneShapeDrawable* mDrawable;

};

//==============================================================================
class PlaneShapeDrawable : public ::osg::ShapeDrawable
{
public:

  PlaneShapeDrawable(kido::dynamics::PlaneShape* shape, PlaneShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~PlaneShapeDrawable();

  kido::dynamics::PlaneShape* mPlaneShape;
  PlaneShapeGeode* mParent;

};

//==============================================================================
PlaneShapeNode::PlaneShapeNode(
    std::shared_ptr<kido::dynamics::PlaneShape> shape,
    EntityNode* parent)
  : ShapeNode(shape, parent, this),
    mPlaneShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void PlaneShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mShape->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void PlaneShapeNode::extractData(bool firstTime)
{
  if(mShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_TRANSFORM)
     || firstTime)
    setMatrix(eigToOsgMatrix(mShape->getLocalTransform()));

  if(nullptr == mGeode)
  {
    mGeode = new PlaneShapeGeode(mPlaneShape.get(), mParentEntity, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
PlaneShapeNode::~PlaneShapeNode()
{
  // Do nothing
}

//==============================================================================
PlaneShapeGeode::PlaneShapeGeode(kido::dynamics::PlaneShape* shape,
                                 EntityNode* parentEntity,
                                 PlaneShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentEntity, this),
    mPlaneShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
  extractData();
}

//==============================================================================
void PlaneShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void PlaneShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new PlaneShapeDrawable(mPlaneShape, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
PlaneShapeGeode::~PlaneShapeGeode()
{
  // Do nothing
}

//==============================================================================
PlaneShapeDrawable::PlaneShapeDrawable(kido::dynamics::PlaneShape* shape,
                                       PlaneShapeGeode* parent)
  : mPlaneShape(shape),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void PlaneShapeDrawable::refresh(bool firstTime)
{
  if(mPlaneShape->getDataVariance() == kido::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if(mPlaneShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    const Eigen::Vector3d& n = mPlaneShape->getNormal();
    const Eigen::Vector3d& p = mPlaneShape->getOffset()*n;
    ::osg::ref_ptr<::osg::InfinitePlane> osg_shape = new ::osg::InfinitePlane;
    static_cast<::osg::Plane&>(*osg_shape) =
        ::osg::Plane(eigToOsgVec3(n), eigToOsgVec3(p));

    setShape(osg_shape);
    dirtyDisplayList();
  }

  if(mPlaneShape->checkDataVariance(kido::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mPlaneShape->getRGBA()));
  }
}

//==============================================================================
PlaneShapeDrawable::~PlaneShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace kido
