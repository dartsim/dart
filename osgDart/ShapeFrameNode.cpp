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

#include "osg/Geode"
#include "osg/Node"
#include "osg/Group"

#include "osgDart/ShapeFrameNode.h"
#include "osgDart/Utils.h"
#include "osgDart/render/ShapeNode.h"
#include "osgDart/render/BoxShapeNode.h"
#include "osgDart/render/EllipsoidShapeNode.h"
#include "osgDart/render/CylinderShapeNode.h"
#include "osgDart/render/PlaneShapeNode.h"
#include "osgDart/render/MeshShapeNode.h"
#include "osgDart/render/SoftMeshShapeNode.h"
#include "osgDart/render/LineSegmentShapeNode.h"
#include "osgDart/render/WarningShapeNode.h"

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

namespace osgDart {

//==============================================================================
ShapeFrameNode::ShapeFrameNode(
    dart::dynamics::ShapeFrame* _frame,
    WorldNode* _worldNode,
    bool _relative, bool _recursive)
  : mShapeFrame(_frame),
    mWorldNode(_worldNode),
    mUtilized(false)
{
  refresh(_relative, _recursive);
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
void ShapeFrameNode::refresh(bool _relative, bool _recursive)
{
  mUtilized = true;

  auto shape = mShapeFrame->getShape();

  if(_relative)
    setMatrix(eigToOsgMatrix(mShapeFrame->getRelativeTransform()));
  else
    setMatrix(eigToOsgMatrix(mShapeFrame->getWorldTransform()));
  // TODO(JS): Maybe the data varicance information should be in ShapeFrame and
  // checked here.

  clearChildUtilizationFlags();

  if(shape)
    refreshShapeNode(shape);

  clearUnusedNodes();

  if(!_recursive)
    return;

  const auto& frames = mShapeFrame->getChildFrames();

  for(auto* frame : frames)
  {
    const auto shapeFrame = dynamic_cast<dart::dynamics::ShapeFrame*>(frame);
    if(shapeFrame)
      refreshShapeFrameNode(shapeFrame);
  }
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
void ShapeFrameNode::clearChildUtilizationFlags()
{
  for(auto& node : mNodeToShape)
    node.first->clearUtilization();
}

//==============================================================================
void ShapeFrameNode::clearUnusedNodes()
{
  for(auto node_pair_it = mNodeToShape.begin();
      node_pair_it != mNodeToShape.end(); )
  {
    render::ShapeNode* node = node_pair_it->first;

    if(!node->wasUtilized())
    {
      auto shape = node_pair_it->second;

      node_pair_it = mNodeToShape.erase(node_pair_it);
      mShapeToNode.erase(shape);

      removeChild(node->getNode());
    }
    else
    {
      ++node_pair_it;
    }
  }
}

//==============================================================================
void ShapeFrameNode::refreshShapeFrameNode(
    dart::dynamics::ShapeFrame* shapeFrame)
{
  auto it = mShapeFrameToNode.find(shapeFrame);

  if(it == mShapeFrameToNode.end())
  {
    createShapeFrameNode(shapeFrame);
    return;
  }

  (it->second)->refresh(true, true);
}

//==============================================================================
void ShapeFrameNode::createShapeFrameNode(
    dart::dynamics::ShapeFrame* shapeFrame)
{
  osg::ref_ptr<ShapeFrameNode> node =
      new ShapeFrameNode(shapeFrame, mWorldNode, true, true);

  mShapeFrameToNode[shapeFrame] = node.get();
  mNodeToShapeFrame[node.get()] = shapeFrame;

  addChild(node);
}

//==============================================================================
void ShapeFrameNode::refreshShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  auto it = mShapeToNode.find(shape);

  if(it == mShapeToNode.end())
  {
    createShapeNode(shape);
    return;
  }

  (it->second)->refresh();
}

//==============================================================================
static void warnAboutUnsuccessfulCast(const std::string& shapeType,
                                      const std::string& entityName)
{
  dtwarn << "[osgDart::EntityNode::createShapeNode] A Shape in '" << entityName
         << "' claimed to be a '" << shapeType << "' but it failed to be "
         << "dynamically cast to that type. "
         << "It will not be added to the OSG tree, "
         << "and therefore will not be rendered\n";
}

//==============================================================================
void ShapeFrameNode::createShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  using namespace dart::dynamics;
  render::ShapeNode* node = nullptr;

  switch(shape->getShapeType())
  {
    case Shape::BOX:
    {
      std::shared_ptr<BoxShape> bs =
          std::dynamic_pointer_cast<BoxShape>(shape);
      if(bs)
      {
        auto boxShapeNode = new render::BoxShapeNode(bs, this);
        addChild(boxShapeNode);
        node = boxShapeNode;
      }
      else
        warnAboutUnsuccessfulCast("BoxShape", mShapeFrame->getName());
      break;
    }

    case Shape::ELLIPSOID:
    {
      std::shared_ptr<EllipsoidShape> es =
          std::dynamic_pointer_cast<EllipsoidShape>(shape);
      if(es)
        node = new render::EllipsoidShapeNode(es, this);
      else
        warnAboutUnsuccessfulCast("EllipsoidShape", mShapeFrame->getName());
      break;
    }

    case Shape::CYLINDER:
    {
      std::shared_ptr<CylinderShape> cs =
          std::dynamic_pointer_cast<CylinderShape>(shape);
      if(cs)
        node = new render::CylinderShapeNode(cs, this);
      else
        warnAboutUnsuccessfulCast("CylinderShape", mShapeFrame->getName());
      break;
    }

    case Shape::PLANE:
    {
      std::shared_ptr<PlaneShape> ps =
          std::dynamic_pointer_cast<PlaneShape>(shape);
      if(ps)
        node = new render::PlaneShapeNode(ps, this);
      else
        warnAboutUnsuccessfulCast("PlaneShape", mShapeFrame->getName());
      break;
    }

    case Shape::MESH:
    {
      std::shared_ptr<MeshShape> ms =
          std::dynamic_pointer_cast<MeshShape>(shape);
      if(ms)
        node = new render::MeshShapeNode(ms, this);
      else
        warnAboutUnsuccessfulCast("MeshShape", mShapeFrame->getName());
      break;
    }

    case Shape::SOFT_MESH:
    {
      std::shared_ptr<SoftMeshShape> sms =
          std::dynamic_pointer_cast<SoftMeshShape>(shape);
      if(sms)
        node = new render::SoftMeshShapeNode(sms, this);
      else
        warnAboutUnsuccessfulCast("SoftMeshShape", mShapeFrame->getName());
      break;
    }

    case Shape::LINE_SEGMENT:
    {
      std::shared_ptr<LineSegmentShape> lss =
          std::dynamic_pointer_cast<LineSegmentShape>(shape);
      if(lss)
        node = new render::LineSegmentShapeNode(lss, this);
      else
        warnAboutUnsuccessfulCast("LineSegmentShape", mShapeFrame->getName());
      break;
    }

    default:
      node = new render::WarningShapeNode(shape, this);
      break;
  }

  if(nullptr == node)
    return;

  mShapeToNode[shape] = node;
  mNodeToShape[node] = shape;

  addChild(node->getNode());
}

} // namespace osgDart
