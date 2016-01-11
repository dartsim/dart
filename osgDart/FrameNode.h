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

#ifndef OSGKIDO_FRAMENODE_H
#define OSGKIDO_FRAMENODE_H

#include <osg/MatrixTransform>
#include <map>

namespace kido {
namespace dynamics {
class Frame;
class Entity;
} // namespace dynamics
} // namespace kido

namespace osgDart
{

class WorldNode;
class EntityNode;

class FrameNode : public osg::MatrixTransform
{
public:

  /// Create a FrameNode. If _recursive is set to true, it will also create
  /// nodes for all child Entities and child Frames
  FrameNode(kido::dynamics::Frame* _frame, WorldNode* _worldNode,
            bool _relative, bool _recursive);

  /// Pointer to the Frame associated with this FrameNode
  kido::dynamics::Frame* getFrame() const;

  WorldNode* getWorldNode();

  const WorldNode* getWorldNode() const;

  /// Update all rendering data for this Frame
  ///
  /// If _recursive is set to true, this FrameNode will also trigger refreshing
  /// on all child Entities and child Frames
  void refresh(bool _relative, bool _recursive);

  /// True iff this FrameNode has been utilized on the latest update
  bool wasUtilized() const;

  /// Set mUtilized to false
  void clearUtilization();

protected:

  virtual ~FrameNode();

  void clearChildUtilizationFlags();

  void clearUnusedNodes();

  void refreshFrameNode(kido::dynamics::Frame* _frame);

  void createFrameNode(kido::dynamics::Frame* _frame);

  void refreshEntityNode(kido::dynamics::Entity* _entity);

  void createEntityNode(kido::dynamics::Entity* _entity);

  /// Pointer to the Frame that this FrameNode is associated with
  kido::dynamics::Frame* mFrame;

  /// Pointer to the WorldNode that this FrameNode belongs to
  WorldNode* mWorldNode;

  /// Map from child Entities to child EntityNodes
  std::map<kido::dynamics::Entity*, EntityNode*> mEntityToNode;

  /// Map from child EntityNodes to child Entities
  std::map<EntityNode*, kido::dynamics::Entity*> mNodeToEntity;

  /// Map from child Frames to child FrameNodes
  std::map<kido::dynamics::Frame*, FrameNode*> mFrameToNode;

  /// Map from child FrameNodes to child Frames
  std::map<FrameNode*, kido::dynamics::Frame*> mNodeToFrame;

  /// True iff this FrameNode has been utilized on the latest update.
  /// If it has not, that is an indication that it is no longer being
  /// used and should be deleted.
  bool mUtilized;

};

} // namespace osgDart

#endif // OSGKIDO_FRAMENODE_H
