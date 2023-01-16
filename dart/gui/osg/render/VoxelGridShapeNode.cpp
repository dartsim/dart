/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#if DART_HAVE_OCTOMAP

  #include "dart/dynamics/SimpleFrame.hpp"
  #include "dart/dynamics/VoxelGridShape.hpp"
  #include "dart/gui/osg/Utils.hpp"

  #include <osg/CullFace>
  #include <osg/Depth>
  #include <osg/Geode>
  #include <osg/Light>
  #include <osg/Material>
  #include <osg/ShapeDrawable>

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class BoxDrawable final : public ::osg::ShapeDrawable
{
public:
  BoxDrawable(double size, const Eigen::Vector4d& color)
  {
    mShape = new ::osg::Box(::osg::Vec3(), static_cast<float>(size));
    setColor(eigToOsgVec4f(color));
    setShape(mShape);
    setDataVariance(::osg::Object::DYNAMIC);
    getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
    getOrCreateStateSet()->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
    getOrCreateStateSet()->setAttributeAndModes(
        new ::osg::CullFace(::osg::CullFace::BACK));
  }

  void updateSize(double size)
  {
    mShape->setHalfLengths(::osg::Vec3(
        static_cast<float>(size * 0.5),
        static_cast<float>(size * 0.5),
        static_cast<float>(size * 0.5)));
    dirtyBound();
    dirtyDisplayList();
  }

  void updateColor(const Eigen::Vector4d& color)
  {
    // Set color
    setColor(eigToOsgVec4f(color));

    // Set alpha specific properties
    ::osg::StateSet* ss = getOrCreateStateSet();
    if (std::abs(color[3]) > 1 - getAlphaThreshold()) {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    } else {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
      ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(false);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
  }

protected:
  ::osg::ref_ptr<::osg::Box> mShape;
};

//==============================================================================
class VoxelNode : public ::osg::MatrixTransform
{
public:
  VoxelNode(
      const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
  {
    mDrawable = new BoxDrawable(size, color);
    mGeode = new ::osg::Geode();

    mGeode->addDrawable(mDrawable);
    addChild(mGeode);

    updateCenter(point);
  }

  void updateCenter(const Eigen::Vector3d& point)
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = point;
    setMatrix(eigToOsgMatrix(tf));
  }

  void updateSize(double size)
  {
    mDrawable->updateSize(size);
  }

  void updateColor(const Eigen::Vector4d& color)
  {
    mDrawable->updateColor(color);
  }

protected:
  ::osg::ref_ptr<BoxDrawable> mDrawable;
  ::osg::ref_ptr<::osg::Geode> mGeode;
};

//==============================================================================
VoxelGridShapeNode::VoxelGridShapeNode(
    std::shared_ptr<dynamics::VoxelGridShape> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mVoxelGridShape(shape),
    mGeode(nullptr),
    mVoxelGridVersion(dynamics::INVALID_INDEX)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);
}

//==============================================================================
void VoxelGridShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC
      && mVoxelGridVersion == mVoxelGridShape->getVersion()) {
    return;
  }

  extractData(false);

  mVoxelGridVersion = mVoxelGridShape->getVersion();
}

//==============================================================================
Eigen::Vector3d toVector3d(const octomap::point3d& point)
{
  return Eigen::Vector3d(
      static_cast<double>(point.x()),
      static_cast<double>(point.y()),
      static_cast<double>(point.z()));
}

//==============================================================================
void VoxelGridShapeNode::extractData(bool /*firstTime*/)
{
  auto tree = mVoxelGridShape->getOctree();
  const auto visualSize = tree->getResolution();
  const auto& color = mVisualAspect->getRGBA();

  // Pre-allocate for the case that the size of new points are greater than
  // previous update
  const auto newVoxels = tree->getNumLeafNodes();
  mVoxelNodes.reserve(newVoxels);

  // Update position of cache boxes.
  std::size_t boxIndex = 0u;
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end;
       ++it) {
    auto threashold = tree->getOccupancyThres();

    if (it->getOccupancy() < threashold)
      continue;

    if (boxIndex < mVoxelNodes.size()) {
      mVoxelNodes[boxIndex]->updateCenter(toVector3d(it.getCoordinate()));
      mVoxelNodes[boxIndex]->updateSize(visualSize);
      mVoxelNodes[boxIndex]->updateColor(color);
    } else {
      ::osg::ref_ptr<VoxelNode> voxelNode
          = new VoxelNode(toVector3d(it.getCoordinate()), visualSize, color);
      mVoxelNodes.emplace_back(voxelNode);
      addChild(mVoxelNodes.back());
    }

    boxIndex++;
  }

  // Fit the size of cache box list to the new points. No effect new boxes are
  // added to the list.
  if (mVoxelNodes.size() > boxIndex) {
    removeChildren(
        static_cast<unsigned int>(boxIndex),
        static_cast<unsigned int>(mVoxelNodes.size() - boxIndex));
    mVoxelNodes.resize(boxIndex);
  }
}

//==============================================================================
VoxelGridShapeNode::~VoxelGridShapeNode()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_HAVE_OCTOMAP
