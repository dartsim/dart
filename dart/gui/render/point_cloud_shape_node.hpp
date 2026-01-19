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

#ifndef DART_GUI_RENDER_POINTCLOUDSHAPENODE_HPP_
#define DART_GUI_RENDER_POINTCLOUDSHAPENODE_HPP_

#include <dart/gui/render/shape_node.hpp>

#include <dart/dynamics/point_cloud_shape.hpp>

#include <Eigen/Dense>
#include <osg/Group>

namespace dart {

namespace dynamics {
class PointCloudShape;
} // namespace dynamics

namespace gui {

namespace render {

class PointCloudShapeGeode;
class PointCloudShapeBillboardGeode;
class PointNode;

class PointNodes : public ::osg::Group
{
public:
  PointNodes() = default;
  virtual ~PointNodes() override = default;

  virtual void refresh(bool firstTime) = 0;
};

class DART_GUI_API PointCloudShapeNode : public ShapeNode, public ::osg::Group
{
public:
  PointCloudShapeNode(
      std::shared_ptr<dart::dynamics::PointCloudShape> shape,
      ShapeFrameNode* parent);

  void refresh() override;
  void extractData(bool firstTime);

  ::osg::ref_ptr<PointNodes> createPointNodes();

protected:
  virtual ~PointCloudShapeNode() override;

  std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape;
  ::osg::ref_ptr<PointNodes> mPointNodes;
  std::size_t mPointCloudVersion;

  dynamics::PointCloudShape::PointShapeType mPointShapeType;
};

} // namespace render
} // namespace gui
} // namespace dart

#endif // DART_GUI_RENDER_POINTCLOUDSHAPENODE_HPP_
