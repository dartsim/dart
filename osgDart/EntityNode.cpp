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

#include "osgDart/EntityNode.h"
#include "osgDart/render/ShapeNode.h"
#include "osgDart/render/BoxShapeNode.h"
#include "osgDart/render/EllipsoidShapeNode.h"

#include "dart/dynamics/Entity.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"

namespace osgDart {

EntityNode::EntityNode(dart::dynamics::Entity* _entity, FrameNode* _parent)
  : mEntity(_entity),
    mParent(_parent),
    mUtilized(false)
{
  refresh();
}

//==============================================================================
dart::dynamics::Entity* EntityNode::getEntity() const
{
  return mEntity;
}

//==============================================================================
FrameNode* EntityNode::getParentFrameNode()
{
  return mParent;
}

//==============================================================================
const FrameNode* EntityNode::getParentFrameNode() const
{
  return mParent;
}

//==============================================================================
void EntityNode::refresh()
{
  mUtilized = true;

  const std::vector<dart::dynamics::Shape*>& visShapes =
      mEntity->getVisualizationShapes();

  for(dart::dynamics::Shape* shape : visShapes)
    refreshShapeNode(shape);
}

//==============================================================================
bool EntityNode::wasUtilized() const
{
  return mUtilized;
}

//==============================================================================
void EntityNode::clearUtilization()
{
  mUtilized = false;
}

//==============================================================================
EntityNode::~EntityNode()
{
  // Do nothing
}

//==============================================================================
void EntityNode::clearChildUtilizationFlags()
{
  for(auto& node : mNodeToShape)
    node.first->clearUtilization();
}

//==============================================================================
void EntityNode::clearUnusedNodes()
{
  for(auto& node_pair : mNodeToShape)
  {
    render::ShapeNode* node = node_pair.first;
    if(!node->wasUtilized())
    {
      mNodeToShape.erase(node);
      mShapeToNode.erase(node_pair.second);

      removeChild(node->getNode());
    }
  }
}

//==============================================================================
void EntityNode::refreshShapeNode(dart::dynamics::Shape* shape)
{
  std::map<dart::dynamics::Shape*, render::ShapeNode*>::iterator it =
      mShapeToNode.find(shape);

  if(it == mShapeToNode.end())
  {
    createShapeNode(shape);
    return;
  }

  (it->second)->refresh();
}

//==============================================================================
void EntityNode::createShapeNode(dart::dynamics::Shape* shape)
{
  using namespace dart::dynamics;
  render::ShapeNode* node = nullptr;

  switch(shape->getShapeType())
  {
    case Shape::BOX:
    {
      BoxShape* bs = dynamic_cast<BoxShape*>(shape);
      if(bs)
        node = new render::BoxShapeNode(bs, this);
      break;
    }
    case Shape::ELLIPSOID:
    {
      EllipsoidShape* es = dynamic_cast<EllipsoidShape*>(shape);
      if(es)
        node = new render::EllipsoidShapeNode(es, this);
      break;
    }
    default:
      break;
  }

  if(nullptr == node)
    return;

  mShapeToNode[shape] = node;
  mNodeToShape[node] = shape;

  addChild(node->getNode());
}

} // namespace osgDart
