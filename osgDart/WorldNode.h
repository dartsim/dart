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

#ifndef OSGDART_WORLDNODE_H
#define OSGDART_WORLDNODE_H

#include <osg/Group>
#include <map>

namespace dart {

namespace simulation {
class World;
} // namespace simulation

namespace dynamics {
class Frame;
class Entity;
}

} // namespace dart

namespace osgDart
{

class FrameNode;
class EntityNode;

/// WorldNode class encapsulates a World to be displayed in OpenSceneGraph
class WorldNode : public osg::Group
{
public:

  explicit WorldNode(dart::simulation::World* _world = nullptr);

  /// Set the World that this WorldNode is associated with
  void setWorld(dart::simulation::World* _newWorld);

  /// Get the World that this WorldNode is associated with
  dart::simulation::World* getWorld() const;

  /// This function is called at the beginning of each rendering cycle. It
  /// updates the tree of Frames and Entities that need to be rendered. It may
  /// also take a simulation step if the simulation is not paused.
  ///
  /// If you want to customize what happens at the beginning of each rendering
  /// cycle, you can either overload this function, or you can overload
  /// customUpdate(). This update() function will automatically call
  /// customUpdate() at the beginning of each rendering cycle. By default,
  /// customUpdate() does nothing.
  virtual void update();

  /// If update() is not overloaded, this function will be called at the
  /// beginning of each rendering cycle. This function can be overloaded to
  /// customize the behavior of each update. The default behavior is to do
  /// nothing, so overloading this function will not interfere with the usual
  /// update() operation.
  virtual void customUpdate();

protected:

  virtual ~WorldNode();

  void clearUtilizationFlags();

  void clearUnusedNodes();

  void refreshSkeletons();

  void refreshCustomFrames();

  void refreshCustomEntities();

  void refreshBaseFrameNode(dart::dynamics::Frame* _frame);

  void createBaseFrameNode(dart::dynamics::Frame* _frame);

  void refreshBaseEntityNode(dart::dynamics::Entity* _entity);

  void createBaseEntityNode(dart::dynamics::Entity* _entity);

  std::map<dart::dynamics::Frame*, FrameNode*> mFrameToNode;
  std::map<FrameNode*, dart::dynamics::Frame*> mNodeToFrame;

  std::map<dart::dynamics::Entity*, EntityNode*> mEntityToNode;
  std::map<EntityNode*, dart::dynamics::Entity*> mNodeToEntity;


  dart::simulation::World* mWorld;

};

} // namespace osgDart

#endif // OSGDART_WORLDNODE_H
