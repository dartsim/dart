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

#include "dart/gui/osg/render/ShapeNode.hpp"

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

ShapeNode::ShapeNode(
    std::shared_ptr<dart::dynamics::Shape> shape,
    ShapeFrameNode* parentNode,
    ::osg::Node* node)
  : mShape(std::move(shape)),
    mParentShapeFrameNode(parentNode),
    mNode(node),
    mUtilized(true)
{
  mShapeFrame = mParentShapeFrameNode->getShapeFrame();
  mVisualAspect = mShapeFrame->getVisualAspect();
  assert(mVisualAspect);
}

//==============================================================================
ShapeNode::~ShapeNode()
{
  // Do nothing
}

//==============================================================================
const std::shared_ptr<dart::dynamics::Shape>& ShapeNode::getShape() const
{
  return mShape;
}

//==============================================================================
const dart::dynamics::ShapeFrame* ShapeNode::getShapeFrame() const
{
  return mShapeFrame;
}

//==============================================================================
dart::dynamics::VisualAspect* ShapeNode::getVisualAspect()
{
  return mVisualAspect;
}

//==============================================================================
const dart::dynamics::VisualAspect* ShapeNode::getVisualAspect() const
{
  return mVisualAspect;
}

//==============================================================================
::osg::Node* ShapeNode::getNode()
{
  return mNode;
}

//==============================================================================
const ::osg::Node* ShapeNode::getNode() const
{
  return mNode;
}

//==============================================================================
ShapeFrameNode* ShapeNode::getParentShapeFrameNode()
{
  return mParentShapeFrameNode;
}

//==============================================================================
const ShapeFrameNode* ShapeNode::getParentShapeFrameNode() const
{
  return mParentShapeFrameNode;
}

//==============================================================================
bool ShapeNode::wasUtilized() const
{
  return mUtilized;
}

//==============================================================================
void ShapeNode::clearUtilization()
{
  mUtilized = false;
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
