/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "osg/Geode"
#include "osg/Node"
#include "osg/Group"

#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"
#include "dart/gui/osg/render/ShapeNode.hpp"
#include "dart/gui/osg/render/SphereShapeNode.hpp"
#include "dart/gui/osg/render/BoxShapeNode.hpp"
#include "dart/gui/osg/render/EllipsoidShapeNode.hpp"
#include "dart/gui/osg/render/CylinderShapeNode.hpp"
#include "dart/gui/osg/render/CapsuleShapeNode.hpp"
#include "dart/gui/osg/render/ConeShapeNode.hpp"
#include "dart/gui/osg/render/PlaneShapeNode.hpp"
#include "dart/gui/osg/render/MultiSphereShapeNode.hpp"
#include "dart/gui/osg/render/MeshShapeNode.hpp"
#include "dart/gui/osg/render/SoftMeshShapeNode.hpp"
#include "dart/gui/osg/render/LineSegmentShapeNode.hpp"
#include "dart/gui/osg/render/WarningShapeNode.hpp"

#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Entity.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

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

  const auto& shapeType = shape->getType();

  if(SphereShape::getStaticType() == shapeType)
  {
    std::shared_ptr<SphereShape> es =
        std::dynamic_pointer_cast<SphereShape>(shape);
    if(es)
      mShapeNode = new render::SphereShapeNode(es, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(BoxShape::getStaticType() == shapeType)
  {
    std::shared_ptr<BoxShape> bs =
        std::dynamic_pointer_cast<BoxShape>(shape);
    if(bs)
      mShapeNode = new render::BoxShapeNode(bs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(EllipsoidShape::getStaticType() == shapeType)
  {
    std::shared_ptr<EllipsoidShape> es =
        std::dynamic_pointer_cast<EllipsoidShape>(shape);
    if(es)
      mShapeNode = new render::EllipsoidShapeNode(es, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(CylinderShape::getStaticType() == shapeType)
  {
    std::shared_ptr<CylinderShape> cs =
        std::dynamic_pointer_cast<CylinderShape>(shape);
    if(cs)
      mShapeNode = new render::CylinderShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(CapsuleShape::getStaticType() == shapeType)
  {
    std::shared_ptr<CapsuleShape> cs =
        std::dynamic_pointer_cast<CapsuleShape>(shape);
    if(cs)
      mShapeNode = new render::CapsuleShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(ConeShape::getStaticType() == shapeType)
  {
    std::shared_ptr<ConeShape> cs =
        std::dynamic_pointer_cast<ConeShape>(shape);
    if(cs)
      mShapeNode = new render::ConeShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(PlaneShape::getStaticType() == shapeType)
  {
    std::shared_ptr<PlaneShape> ps =
        std::dynamic_pointer_cast<PlaneShape>(shape);
    if(ps)
      mShapeNode = new render::PlaneShapeNode(ps, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(shape->is<MultiSphereShape>())
  {
    std::shared_ptr<MultiSphereShape> ms =
        std::dynamic_pointer_cast<MultiSphereShape>(shape);
    if(ms)
      mShapeNode = new render::MultiSphereShapeNode(ms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(MeshShape::getStaticType() == shapeType)
  {
    std::shared_ptr<MeshShape> ms =
        std::dynamic_pointer_cast<MeshShape>(shape);
    if(ms)
      mShapeNode = new render::MeshShapeNode(ms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(SoftMeshShape::getStaticType() == shapeType)
  {
    std::shared_ptr<SoftMeshShape> sms =
        std::dynamic_pointer_cast<SoftMeshShape>(shape);
    if(sms)
      mShapeNode = new render::SoftMeshShapeNode(sms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else if(LineSegmentShape::getStaticType() == shapeType)
  {
    std::shared_ptr<LineSegmentShape> lss =
        std::dynamic_pointer_cast<LineSegmentShape>(shape);
    if(lss)
      mShapeNode = new render::LineSegmentShapeNode(lss, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  else
  {
    mShapeNode = new render::WarningShapeNode(shape, this);
  }

  if(nullptr == mShapeNode)
    return;

  addChild(mShapeNode->getNode());
}

} // namespace osg
} // namespace gui
} // namespace dart
