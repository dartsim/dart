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

#ifndef DART_GUI_OSG_RENDER_SHAPEGEODE_HPP_
#define DART_GUI_OSG_RENDER_SHAPEGEODE_HPP_

#include <memory>
#include <osg/Node>

namespace dart {

namespace dynamics {
class Shape;
class ShapeFrame;
class SimpleFrame;
class VisualAspect;
} // namespace dynamics

namespace gui {
namespace osg {

class Node;
class Group;
class EntityNode;
class ShapeFrameNode;

namespace render {

class ShapeNode
{
public:

  ShapeNode(std::shared_ptr<dart::dynamics::Shape> shape,
            ShapeFrameNode* parentNode,
            ::osg::Node* node);

  virtual ~ShapeNode();

  /// Pointer to the Shape associated with this ShapeNode
  const std::shared_ptr<dart::dynamics::Shape>& getShape() const;

  const dart::dynamics::ShapeFrame* getShapeFrame() const;

  dart::dynamics::VisualAspect* getVisualAspect();

  const dart::dynamics::VisualAspect* getVisualAspect() const;

  /// Cast this ShapeNode into an osg::Node
  ::osg::Node* getNode();

  /// Cast this ShapeNode into an osg::Node
  const ::osg::Node* getNode() const;

  /// Pointer to the parent EntityNode of this ShapeNode
  ShapeFrameNode* getParentShapeFrameNode();

  /// Pointer to the parent EntityNode of this ShapeNode
  const ShapeFrameNode* getParentShapeFrameNode() const;

  /// Update all rendering data for this ShapeNode
  virtual void refresh() = 0;

  /// True iff this ShapeNode has been utilized on the latest update
  bool wasUtilized() const;

  /// Set mUtilized to false
  void clearUtilization();

protected:
  /// Pointer to the Shape associated with this ShapeNode
  const std::shared_ptr<dart::dynamics::Shape> mShape;

  /// Pointer to the SimpleFrame associated with this ShapeNode
  dart::dynamics::ShapeFrame* mShapeFrame;

  /// Pointer to the VisualAspect associated with this ShapeNode
  dart::dynamics::VisualAspect* mVisualAspect;

  /// Pointer to the parent ShapeFrameNode of this ShapeNode
  ShapeFrameNode* mParentShapeFrameNode;

  /// Should generally be equal to 'this'
  ::osg::Node* mNode;

  /// True iff this ShapeNode has been utilized on the latest update. If it has
  /// not, that is an indication that it is no longer being used and should be
  /// deleted.
  bool mUtilized;

};

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_RENDER_SHAPEGEODE_HPP_
