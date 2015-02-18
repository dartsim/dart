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

#ifndef OSGDART_FRAMENODE_H
#define OSGDART_FRAMENODE_H

#include <osg/MatrixTransform>
#include <map>

namespace dart {
namespace dynamics {
class Frame;
class Entity;
} // namespace dynamics
} // namespace dart

namespace osgDart
{

class EntityNode;

class FrameNode : public osg::MatrixTransform
{
public:

  FrameNode(dart::dynamics::Frame* _frame);

  /// Pointer to the Frame associated with this FrameNode
  dart::dynamics::Frame* frame();

  /// Pointer to the Frame associated with this FrameNode
  const dart::dynamics::Frame* frame() const;

protected:
  /// Pointer to the Frame that this FrameNode is associated with
  dart::dynamics::Frame* mFrame;

  /// Pointer to the EntityNode of this Frame's own shapes
  EntityNode* mPersonalEntityNode;

  /// Map from child Entities to child EntityNodes
  std::map<dart::dynamics::Entity*, EntityNode*> mEntityToNode;

  /// Map from child EntityNodes to child Entities
  std::map<EntityNode*, dart::dynamics::Entity*> mNodeToEntity;

  /// Map from child Frames to child FrameNodes
  std::map<dart::dynamics::Frame*, FrameNode*> mFrameToNode;

  /// Map from child FrameNodes to child Frames
  std::map<FrameNode*, dart::dynamics::Frame*> mNodeToFrame;

};

} // namespace osgDart

#endif // OSGDART_FRAMENODE_H
