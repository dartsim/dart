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

#ifndef DART_GUI_OSG_RENDER_VOXELGRIDSHAPENODE_HPP_
#define DART_GUI_OSG_RENDER_VOXELGRIDSHAPENODE_HPP_

#include <dart/config.hpp>

#if HAVE_OCTOMAP

  #include <dart/gui/osg/render/ShapeNode.hpp>

  #include <osg/MatrixTransform>
  #include <osg/ShapeDrawable>

namespace dart {

namespace dynamics {
class VoxelGridShape;
} // namespace dynamics

namespace gui {
namespace osg {
namespace render {

class VoxelGridShapeGeode;
class VoxelNode;

class VoxelGridShapeNode : public ShapeNode, public ::osg::Group
{
public:
  VoxelGridShapeNode(
      std::shared_ptr<dynamics::VoxelGridShape> shape, ShapeFrameNode* parent);

  void refresh() override;
  void extractData(bool firstTime);

protected:
  virtual ~VoxelGridShapeNode() override;

  std::shared_ptr<dynamics::VoxelGridShape> mVoxelGridShape;
  VoxelGridShapeGeode* mGeode;
  std::vector<::osg::ref_ptr<VoxelNode>> mVoxelNodes;
  std::size_t mVoxelGridVersion;
};

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // HAVE_OCTOMAP

#endif // DART_GUI_OSG_RENDER_VOXELGRIDSHAPENODE_HPP_
