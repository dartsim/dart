/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/gui/osg/render/VoxelGridShapeNode.hpp"

#include <osg/Geode>
#include <osg/Light>
#include <osg/Material>
#include <osg/ShapeDrawable>

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/VoxelGridShape.hpp"
#include "dart/gui/osg/Utils.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class VoxelGridShapeDrawable : public ::osg::ShapeDrawable
{
public:
  VoxelGridShapeDrawable(
      dynamics::VoxelGridShape* shape,
      dynamics::VisualAspect* visualAspect,
      VoxelGridShapeGeode* parent);

  void refresh(bool firstTime);

protected:
  ~VoxelGridShapeDrawable() override = default;

  dynamics::VoxelGridShape* mVoxelGridShape;
  dynamics::VisualAspect* mVisualAspect;
  VoxelGridShapeGeode* mParent;
  std::size_t mVoxelVersion;

private:
  void updateBoxes(
      ::osg::CompositeShape* osgShape,
      const octomap::OcTree* tree,
      double threashold);
  std::vector<::osg::ref_ptr<::osg::Box>> mBoxes;
};

//==============================================================================
class VoxelGridShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  VoxelGridShapeGeode(
      dynamics::VoxelGridShape* shape,
      ShapeFrameNode* parentShapeFrame,
      VoxelGridShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~VoxelGridShapeGeode();

  VoxelGridShapeNode* mParentNode;
  dynamics::VoxelGridShape* mVoxelGridShape;
  VoxelGridShapeDrawable* mDrawable;
};

//==============================================================================
VoxelGridShapeNode::VoxelGridShapeNode(
    std::shared_ptr<dynamics::VoxelGridShape> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mVoxelGridShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);
}

//==============================================================================
void VoxelGridShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);

  if (mShape->getDataVariance() == dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void VoxelGridShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new VoxelGridShapeGeode(
        mVoxelGridShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
VoxelGridShapeNode::~VoxelGridShapeNode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeGeode::VoxelGridShapeGeode(
    dynamics::VoxelGridShape* shape,
    ShapeFrameNode* parentShapeFrame,
    VoxelGridShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mVoxelGridShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void VoxelGridShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void VoxelGridShapeGeode::extractData()
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new VoxelGridShapeDrawable(mVoxelGridShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
VoxelGridShapeGeode::~VoxelGridShapeGeode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeDrawable::VoxelGridShapeDrawable(
    dynamics::VoxelGridShape* shape,
    dynamics::VisualAspect* visualAspect,
    VoxelGridShapeGeode* parent)
  : mVoxelGridShape(shape),
    mVisualAspect(visualAspect),
    mParent(parent),
    mVoxelVersion(dynamics::INVALID_INDEX)
{
  refresh(true);
}

//==============================================================================
::osg::Vec3 toVec3(const octomap::point3d& point)
{
  return ::osg::Vec3(point.x(), point.y(), point.z());
}

//==============================================================================
void VoxelGridShapeDrawable::refresh(bool firstTime)
{
  if (mVoxelGridShape->getDataVariance() == dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mVoxelGridShape->checkDataVariance(dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime)
  {
    if (mVoxelVersion != mVoxelGridShape->getVersion())
    {
      auto osgShape = new ::osg::CompositeShape();
      auto octomap = mVoxelGridShape->getOctree();
      updateBoxes(osgShape, octomap.get(), 0.75);

      setShape(osgShape);
      dirtyDisplayList();
    }
  }

  if (mVoxelGridShape->checkDataVariance(dynamics::Shape::DYNAMIC_COLOR)
      || firstTime)
  {
    setColor(eigToOsgVec4(mVisualAspect->getRGBA()));
  }

  mVoxelVersion = mVoxelGridShape->getVersion();
}

//==============================================================================
void VoxelGridShapeDrawable::updateBoxes(
    ::osg::CompositeShape* osgShape,
    const octomap::OcTree* tree,
    double threashold)
{
  const auto size = static_cast<float>(tree->getResolution());
  const auto newNumBoxes = tree->getNumLeafNodes();
  mBoxes.reserve(newNumBoxes);

  // TODO(JS): Use begin_leafs_bbx/end_leafs_bbx to only render voxels that
  // are in the view sight. For this, camera view frustum would be required.

  std::size_t boxIndex = 0u;
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
  {
    threashold = tree->getOccupancyThres();

    if (it->getOccupancy() < threashold)
      continue;

    if (boxIndex < mBoxes.size())
    {
      mBoxes[boxIndex]->setCenter(toVec3(it.getCoordinate()));
    }
    else
    {
      auto osgSphere = new ::osg::Box(toVec3(it.getCoordinate()), size);
      mBoxes.emplace_back(osgSphere);
    }

    osgShape->addChild(mBoxes[boxIndex]);

    boxIndex++;
  }

  mBoxes.resize(boxIndex);
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
