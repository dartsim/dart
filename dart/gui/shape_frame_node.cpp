/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/gui/shape_frame_node.hpp"

#include "dart/gui/render/box_shape_node.hpp"
#include "dart/gui/render/capsule_shape_node.hpp"
#include "dart/gui/render/cone_shape_node.hpp"
#include "dart/gui/render/convex_mesh_shape_node.hpp"
#include "dart/gui/render/cylinder_shape_node.hpp"
#include "dart/gui/render/ellipsoid_shape_node.hpp"
#include "dart/gui/render/line_segment_shape_node.hpp"
#include "dart/gui/render/mesh_shape_node.hpp"
#include "dart/gui/render/multi_sphere_shape_node.hpp"
#include "dart/gui/render/plane_shape_node.hpp"
#include "dart/gui/render/point_cloud_shape_node.hpp"
#include "dart/gui/render/pyramid_shape_node.hpp"
#include "dart/gui/render/shape_node.hpp"
#include "dart/gui/render/soft_mesh_shape_node.hpp"
#include "dart/gui/render/sphere_shape_node.hpp"
#include "dart/gui/utils.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Node>
#if DART_HAVE_OCTOMAP
  #include "dart/gui/render/voxel_grid_shape_node.hpp"
#endif
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/entity.hpp"
#include "dart/dynamics/frame.hpp"
#include "dart/dynamics/line_segment_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/point_cloud_shape.hpp"
#include "dart/dynamics/pyramid_shape.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/soft_mesh_shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/gui/render/heightmap_shape_node.hpp"
#include "dart/gui/render/warning_shape_node.hpp"
#if DART_HAVE_OCTOMAP
  #include "dart/dynamics/voxel_grid_shape.hpp"
#endif
#include "dart/dynamics/heightmap_shape.hpp"
#include "dart/dynamics/simple_frame.hpp"

namespace dart {
namespace gui {

//==============================================================================
ShapeFrameNode::ShapeFrameNode(
    dart::dynamics::ShapeFrame* _frame, WorldNode* _worldNode)
  : mShapeFrame(_frame),
    mWorldNode(_worldNode),
    mRenderShapeNode(nullptr),
    mUtilized(false)
{
  refresh();
  setName(_frame->getName() + " [frame]");
}

//==============================================================================
dart::dynamics::ShapeFrame* ShapeFrameNode::getShapeFrame(bool checkUtilization)
{
  DART_WARN_IF(
      !mUtilized && checkUtilization,
      "[ShapeFrameNode] Attempting to access ShapeFrame of unused "
      "ShapeFrameNode. This can be dangerous because unused ShapeFrameNode "
      "implies that it's possible that the ShapeFrame already got deleted.");

  return mShapeFrame;
}

//==============================================================================
const dart::dynamics::ShapeFrame* ShapeFrameNode::getShapeFrame(
    bool checkUtilization) const
{
  DART_WARN_IF(
      !mUtilized && checkUtilization,
      "[ShapeFrameNode] Attempting to access ShapeFrame of unused "
      "ShapeFrameNode. This can be dangerous because unused ShapeFrameNode "
      "implies that it's possible that the ShapeFrame already got deleted.");

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
  if (shortCircuitIfUtilized && mUtilized)
    return;

  mUtilized = true;

  auto shape = mShapeFrame->getShape();

  setMatrix(eigToOsgMatrix(mShapeFrame->getWorldTransform()));
  // TODO(JS): Maybe the data varicance information should be in ShapeFrame and
  // checked here.

  if (shape && mShapeFrame->getVisualAspect()) {
    refreshShapeNode(shape);
  } else if (mRenderShapeNode) {
    removeChild(mRenderShapeNode->getNode());
    mRenderShapeNode = nullptr;
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
void ShapeFrameNode::refreshShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  if (mRenderShapeNode && mRenderShapeNode->getShape() == shape) {
    mRenderShapeNode->refresh();
    return;
  }

  createShapeNode(shape);
}

//==============================================================================
static void warnAboutUnsuccessfulCast(
    std::string_view shapeType, std::string_view entityName)
{
  DART_WARN(
      "A Shape in '{}' claimed to be a '{}' but it failed to be dynamically "
      "cast to that type. It will not be added to the OSG tree, and therefore "
      "will not be rendered",
      entityName,
      shapeType);
}

//==============================================================================
void ShapeFrameNode::createShapeNode(
    const std::shared_ptr<dart::dynamics::Shape>& shape)
{
  using namespace dart::dynamics;
  if (mRenderShapeNode)
    removeChild(mRenderShapeNode->getNode());

  mRenderShapeNode = nullptr;

  const auto& shapeType = shape->getType();

  if (SphereShape::getStaticType() == shapeType) {
    std::shared_ptr<SphereShape> es
        = std::dynamic_pointer_cast<SphereShape>(shape);
    if (es)
      mRenderShapeNode = new render::SphereShapeNode(es, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (BoxShape::getStaticType() == shapeType) {
    std::shared_ptr<BoxShape> bs = std::dynamic_pointer_cast<BoxShape>(shape);
    if (bs)
      mRenderShapeNode = new render::BoxShapeNode(bs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (EllipsoidShape::getStaticType() == shapeType) {
    std::shared_ptr<EllipsoidShape> es
        = std::dynamic_pointer_cast<EllipsoidShape>(shape);
    if (es)
      mRenderShapeNode = new render::EllipsoidShapeNode(es, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (CylinderShape::getStaticType() == shapeType) {
    std::shared_ptr<CylinderShape> cs
        = std::dynamic_pointer_cast<CylinderShape>(shape);
    if (cs)
      mRenderShapeNode = new render::CylinderShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (CapsuleShape::getStaticType() == shapeType) {
    std::shared_ptr<CapsuleShape> cs
        = std::dynamic_pointer_cast<CapsuleShape>(shape);
    if (cs)
      mRenderShapeNode = new render::CapsuleShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (ConeShape::getStaticType() == shapeType) {
    std::shared_ptr<ConeShape> cs = std::dynamic_pointer_cast<ConeShape>(shape);
    if (cs)
      mRenderShapeNode = new render::ConeShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (PyramidShape::getStaticType() == shapeType) {
    std::shared_ptr<PyramidShape> cs
        = std::dynamic_pointer_cast<PyramidShape>(shape);
    if (cs)
      mRenderShapeNode = new render::PyramidShapeNode(cs, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (PlaneShape::getStaticType() == shapeType) {
    std::shared_ptr<PlaneShape> ps
        = std::dynamic_pointer_cast<PlaneShape>(shape);
    if (ps)
      mRenderShapeNode = new render::PlaneShapeNode(ps, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (shape->is<MultiSphereConvexHullShape>()) {
    std::shared_ptr<MultiSphereConvexHullShape> ms
        = std::dynamic_pointer_cast<MultiSphereConvexHullShape>(shape);
    if (ms)
      mRenderShapeNode = new render::MultiSphereShapeNode(ms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (ConvexMeshShape::getStaticType() == shapeType) {
    std::shared_ptr<ConvexMeshShape> cms
        = std::dynamic_pointer_cast<ConvexMeshShape>(shape);
    if (cms)
      mRenderShapeNode = new render::ConvexMeshShapeNode(cms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (MeshShape::getStaticType() == shapeType) {
    std::shared_ptr<MeshShape> ms = std::dynamic_pointer_cast<MeshShape>(shape);
    if (ms)
      mRenderShapeNode = new render::MeshShapeNode(ms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (SoftMeshShape::getStaticType() == shapeType) {
    std::shared_ptr<SoftMeshShape> sms
        = std::dynamic_pointer_cast<SoftMeshShape>(shape);
    if (sms)
      mRenderShapeNode = new render::SoftMeshShapeNode(sms, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (LineSegmentShape::getStaticType() == shapeType) {
    std::shared_ptr<LineSegmentShape> lss
        = std::dynamic_pointer_cast<LineSegmentShape>(shape);
    if (lss)
      mRenderShapeNode = new render::LineSegmentShapeNode(lss, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  } else if (PointCloudShape::getStaticType() == shapeType) {
    std::shared_ptr<PointCloudShape> lss
        = std::dynamic_pointer_cast<PointCloudShape>(shape);
    if (lss)
      mRenderShapeNode = new render::PointCloudShapeNode(lss, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
#if DART_HAVE_OCTOMAP
  else if (VoxelGridShape::getStaticType() == shapeType) {
    std::shared_ptr<VoxelGridShape> lss
        = std::dynamic_pointer_cast<VoxelGridShape>(shape);
    if (lss)
      mRenderShapeNode = new render::VoxelGridShapeNode(lss, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
#endif // DART_HAVE_OCTOMAP
  else if (HeightmapShapef::getStaticType() == shapeType) {
    std::shared_ptr<HeightmapShapef> lss
        = std::dynamic_pointer_cast<HeightmapShapef>(shape);
    if (lss)
      mRenderShapeNode = new render::HeightmapShapeNode<float>(lss, this);
    else
      warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  }
  // OpenSceneGraph currently doesn't support double precision for Heightmap
  // else if(HeightmapShaped::getStaticType() == shapeType)
  // {
  //   std::shared_ptr<HeightmapShaped> lss =
  //       std::dynamic_pointer_cast<HeightmapShaped>(shape);
  //   if(lss)
  //     mRenderShapeNode = new render::HeightmapShapeNode<double>(lss, this);
  //   else
  //     warnAboutUnsuccessfulCast(shapeType, mShapeFrame->getName());
  // }
  else {
    mRenderShapeNode = new render::WarningShapeNode(shape, this);
  }

  if (nullptr == mRenderShapeNode)
    return;

  addChild(mRenderShapeNode->getNode());
}

} // namespace gui
} // namespace dart
