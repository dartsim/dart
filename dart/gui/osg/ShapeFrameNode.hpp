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

#ifndef DART_GUI_OSG_SHAPEFRAMENODE_HPP_
#define DART_GUI_OSG_SHAPEFRAMENODE_HPP_

#include <map>
#include <memory>
#include <osg/MatrixTransform>
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {
class ShapeFrame;
class Entity;
class Shape;
} // namespace dynamics

namespace gui {
namespace osg {

namespace render {
class ShapeNode;
} // namespace render

class WorldNode;

class ShapeFrameNode : public ::osg::MatrixTransform
{
public:

  /// Create a ShapeFrameNode. If recursive is set to true, it will also create
  /// nodes for all child Entities and child Frames
  ShapeFrameNode(dart::dynamics::ShapeFrame* frame,
                 WorldNode* worldNode);

  /// Pointer to the ShapeFrame associated with this ShapeFrameNode
  dart::dynamics::ShapeFrame* getShapeFrame();

  /// Pointer to the ShapeFrame associated with this ShapeFrameNode
  const dart::dynamics::ShapeFrame* getShapeFrame() const;

  WorldNode* getWorldNode();

  const WorldNode* getWorldNode() const;

  /// Update all rendering data for this ShapeFrame
  ///
  /// If shortCircuitIfUtilized is true, this will skip the refresh process if
  /// mUtilized is set to true. clearUtilization() needs to be called before
  /// this function if short circuiting is going to be used.
  void refresh(bool shortCircuitIfUtilized = false);

  /// True iff this ShapeFrameNode has been utilized on the latest update
  bool wasUtilized() const;

  /// Set mUtilized to false
  void clearUtilization();

protected:

  virtual ~ShapeFrameNode();

  void refreshShapeNode(const std::shared_ptr<dart::dynamics::Shape>& shape);

  void createShapeNode(const std::shared_ptr<dart::dynamics::Shape>& shape);

  /// Pointer to the ShapeFrame that this ShapeFrameNode is associated with
  dart::dynamics::ShapeFrame* mShapeFrame;

  /// Pointer to the WorldNode that this ShapeFrameNode belongs to
  WorldNode* mWorldNode;

  render::ShapeNode* mRenderShapeNode;

  /// True iff this ShapeFrameNode has been utilized on the latest update.
  /// If it has not, that is an indication that it is no longer being
  /// used and should be deleted.
  bool mUtilized;

};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_SHAPEFRAMENODE_HPP_
