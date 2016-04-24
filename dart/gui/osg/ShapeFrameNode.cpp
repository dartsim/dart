/*
 * Copyright (c) 2015-2016, Georgia Tech Research Corporation
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

#include "osg/Geode"
#include "osg/Node"
#include "osg/Group"

#include "dart/gui/osg/ShapeFrameNode.h"
#include "dart/gui/osg/Utils.h"
#include "dart/gui/osg/render/ShapeNode.h"
#include "dart/gui/osg/render/BoxShapeNode.h"
#include "dart/gui/osg/render/EllipsoidShapeNode.h"
#include "dart/gui/osg/render/CylinderShapeNode.h"
#include "dart/gui/osg/render/PlaneShapeNode.h"
#include "dart/gui/osg/render/MeshShapeNode.h"
#include "dart/gui/osg/render/SoftMeshShapeNode.h"
#include "dart/gui/osg/render/LineSegmentShapeNode.h"
#include "dart/gui/osg/render/WarningShapeNode.h"

#include "dart/dynamics/Frame.h"
#include "dart/dynamics/ShapeFrame.h"
#include "dart/dynamics/Entity.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/LineSegmentShape.h"
#include "dart/dynamics/SimpleFrame.h"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
ShapeFrameNode::ShapeFrameNode(
    dart::dynamics::ShapeFrame* _frame,
    WorldNode* _worldNode)
  : mShapeFrame(_frame),
    mWorldNode(_worldNode),
    mShapeNode(nullptr),
    mUtilized(false)
{
  refresh();
  setName(_frame->getName()+" [frame]");
}

//==============================================================================
dart::dynamics::ShapeFrame* ShapeFrameNode::getShapeFrame()
{
  return mShapeFrame;
}

//==============================================================================
const dart::dynamics::ShapeFrame* ShapeFrameNode::getShapeFrame() const
{
  return mShapeFrame;
}

//==============================================================================
WorldNode* ShapeFrameNode::getWorldNode()
{
  return mWorldNode;
}

//==============================================================================
const WorldNode* ShapeFrameNode::getWorldNode() const
{
  return mWorldNode;
}

//==============================================================================
void ShapeFrameNode::refresh(bool shortCircuitIfUtilized)
{
  if(shortCircuitIfUtilized && mUtilized)
    return;

  mUtilized = true;

  auto shape = mShapeFrame->getShape();

  setMatrix(eigToOsgMatrix(mShapeFrame->getWorldTransform()));
  // TODO(JS): Maybe the data varicance information should be in ShapeFrame and
  // checked here.

  if(shape)
    refreshShapeNode(shape);
}

//==============================================================================
bool ShapeFrameNode::wasUtilized() const
{
  return mUtilized;
}

//==============================================================================
void ShapeFrameNode::clearUtilization()
{
  mUtilized = false;
}

//==============================================================================
ShapeFrameNode::~ShapeFrameNode()
{
  // Do nothing
}

//==============================================================================
void ShapeFrameNode::refreshShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  if(mShapeNode && mShapeNode->getShape() == shape)
  {
    mShapeNode->refresh();
    return;
  }

  createShapeNode(shape);
}

//==============================================================================
static void warnAboutUnsuccessfulCast(const std::string& shapeType,
                                      const std::string& entityName)
{
  dtwarn << "[dart::gui::osg::EntityNode::createShapeNode] A Shape in '"
         << entityName << "' claimed to be a '" << shapeType
         << "' but it failed to be dynamically cast to that type. "
         << "It will not be added to the OSG tree, "
         << "and therefore will not be rendered\n";
}

//==============================================================================
void ShapeFrameNode::createShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  using namespace dart::dynamics;
  if(mShapeNode)
    removeChild(mShapeNode->getNode());

  mShapeNode = nullptr;

  switch(shape->getShapeType())
  {
    case Shape::BOX:
    {
      std::shared_ptr<BoxShape> bs =
          std::dynamic_pointer_cast<BoxShape>(shape);
      if(bs)
        mShapeNode = new render::BoxShapeNode(bs, this);
      else
        warnAboutUnsuccessfulCast("BoxShape", mShapeFrame->getName());
      break;
    }

    case Shape::ELLIPSOID:
    {
      std::shared_ptr<EllipsoidShape> es =
          std::dynamic_pointer_cast<EllipsoidShape>(shape);
      if(es)
        mShapeNode = new render::EllipsoidShapeNode(es, this);
      else
        warnAboutUnsuccessfulCast("EllipsoidShape", mShapeFrame->getName());
      break;
    }

    case Shape::CYLINDER:
    {
      std::shared_ptr<CylinderShape> cs =
          std::dynamic_pointer_cast<CylinderShape>(shape);
      if(cs)
        mShapeNode = new render::CylinderShapeNode(cs, this);
      else
        warnAboutUnsuccessfulCast("CylinderShape", mShapeFrame->getName());
      break;
    }

    case Shape::PLANE:
    {
      std::shared_ptr<PlaneShape> ps =
          std::dynamic_pointer_cast<PlaneShape>(shape);
      if(ps)
        mShapeNode = new render::PlaneShapeNode(ps, this);
      else
        warnAboutUnsuccessfulCast("PlaneShape", mShapeFrame->getName());
      break;
    }

    case Shape::MESH:
    {
      std::shared_ptr<MeshShape> ms =
          std::dynamic_pointer_cast<MeshShape>(shape);
      if(ms)
        mShapeNode = new render::MeshShapeNode(ms, this);
      else
        warnAboutUnsuccessfulCast("MeshShape", mShapeFrame->getName());
      break;
    }

    case Shape::SOFT_MESH:
    {
      std::shared_ptr<SoftMeshShape> sms =
          std::dynamic_pointer_cast<SoftMeshShape>(shape);
      if(sms)
        mShapeNode = new render::SoftMeshShapeNode(sms, this);
      else
        warnAboutUnsuccessfulCast("SoftMeshShape", mShapeFrame->getName());
      break;
    }

    case Shape::LINE_SEGMENT:
    {
      std::shared_ptr<LineSegmentShape> lss =
          std::dynamic_pointer_cast<LineSegmentShape>(shape);
      if(lss)
        mShapeNode = new render::LineSegmentShapeNode(lss, this);
      else
        warnAboutUnsuccessfulCast("LineSegmentShape", mShapeFrame->getName());
      break;
    }

    default:
      mShapeNode = new render::WarningShapeNode(shape, this);
      break;
  }

  if(nullptr == mShapeNode)
    return;

  addChild(mShapeNode->getNode());
}

} // namespace osg
} // namespace gui
} // namespace dart
