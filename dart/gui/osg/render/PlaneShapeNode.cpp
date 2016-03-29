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

#include "dart/gui/osg/render/PlaneShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

class PlaneShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  PlaneShapeGeode(dart::dynamics::PlaneShape* shape,
                  ShapeFrameNode* parentShapeFrame,
                  PlaneShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~PlaneShapeGeode();

  dart::dynamics::PlaneShape* mPlaneShape;
  PlaneShapeDrawable* mDrawable;

};

//==============================================================================
class PlaneShapeDrawable : public ::osg::ShapeDrawable
{
public:

  PlaneShapeDrawable(dart::dynamics::PlaneShape* shape,
                     dart::dynamics::VisualAddon* visualAddon,
                     PlaneShapeGeode* parent);

  void refresh(bool firstTime);

protected:

  virtual ~PlaneShapeDrawable();

  dart::dynamics::PlaneShape* mPlaneShape;
  dart::dynamics::VisualAddon* mVisualAddon;
  PlaneShapeGeode* mParent;

};

//==============================================================================
PlaneShapeNode::PlaneShapeNode(
    std::shared_ptr<dart::dynamics::PlaneShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mPlaneShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAddon->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void PlaneShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAddon->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void PlaneShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new PlaneShapeGeode(mPlaneShape.get(), mParentShapeFrameNode, this);
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
PlaneShapeGeode::PlaneShapeGeode(dart::dynamics::PlaneShape* shape,
                                 ShapeFrameNode* parentShapeFrame,
                                 PlaneShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mPlaneShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
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
    mDrawable = new PlaneShapeDrawable(mPlaneShape, mVisualAddon, this);
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
PlaneShapeDrawable::PlaneShapeDrawable(dart::dynamics::PlaneShape* shape,
                                       dart::dynamics::VisualAddon* visualAddon,
                                       PlaneShapeGeode* parent)
  : mPlaneShape(shape),
    mVisualAddon(visualAddon),
    mParent(parent)
{
  refresh(true);
}

//==============================================================================
void PlaneShapeDrawable::refresh(bool firstTime)
{
  if(mPlaneShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if(mPlaneShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
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

  if(mPlaneShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAddon->getRGBA()));
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
} // namespace dart
