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

#ifndef OSGKIDO_RENDER_SHAPEGEODE_HPP
#define OSGKIDO_RENDER_SHAPEGEODE_HPP

#include <memory>

namespace osg {
class Node;
} // namespace osg

namespace kido {
namespace dynamics {
class Shape;
} // namespace dynamics
} // namespace kido

namespace osgKido {

class EntityNode;

namespace render {

class ShapeNode
{
public:

  ShapeNode(std::shared_ptr<kido::dynamics::Shape> _shape,
            EntityNode* _parent,
            osg::Node* _node);

  virtual ~ShapeNode();

  /// Pointer to the Shape associated with this ShapeNode
  std::shared_ptr<kido::dynamics::Shape> getShape() const;

  /// Cast this ShapeNode into an osg::Node
  osg::Node* getNode();

  /// Cast this ShapeNode into an osg::Node
  const osg::Node* getNode() const;

  /// Pointer to the parent EntityNode of this ShapeNode
  EntityNode* getParentEntityNode();

  /// Pointer to the parent EntityNode of this ShapeNode
  const EntityNode* getParentEntityNode() const;

  /// Update all rendering data for this ShapeNode
  virtual void refresh() = 0;

  /// True iff this ShapeNode has been utilized on the latest update
  bool wasUtilized() const;

  /// Set mUtilized to false
  void clearUtilization();

protected:
  /// Pointer to the Shape associated with this ShapeNode
  const std::shared_ptr<kido::dynamics::Shape> mShape;

  /// Should generally be equal to 'this'
  osg::Node* const mNode;

  /// Pointer to the parent EntityNode of this ShapeNode
  EntityNode* mParentEntity;

  /// True iff this ShapeNode has been utilized on the latest update. If it has
  /// not, that is an indication that it is no longer being used and should be
  /// deleted.
  bool mUtilized;

};

} // namespace render
} // namespace osgKido

#endif // OSGKIDO_RENDER_SHAPEGEODE_HPP
