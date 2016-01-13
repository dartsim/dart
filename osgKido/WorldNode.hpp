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

#ifndef OSGKIDO_WORLDNODE_H
#define OSGKIDO_WORLDNODE_H

#include <osg/Group>
#include <map>
#include <memory>

#include "osgKido/Viewer.hpp"

namespace kido {

namespace simulation {
class World;
} // namespace simulation

namespace dynamics {
class Frame;
class Entity;
}

} // namespace kido

namespace osgKido
{

class FrameNode;
class EntityNode;
class Viewer;

/// WorldNode class encapsulates a World to be displayed in OpenSceneGraph
class WorldNode : public osg::Group
{
public:

  friend class Viewer;

  /// Default constructor
  explicit WorldNode(std::shared_ptr<kido::simulation::World> _world = nullptr);

  /// Set the World that this WorldNode is associated with
  void setWorld(std::shared_ptr<kido::simulation::World> _newWorld);

  /// Get the World that this WorldNode is associated with
  std::shared_ptr<kido::simulation::World> getWorld() const;

  /// This function is called at the beginning of each rendering cycle. It
  /// updates the tree of Frames and Entities that need to be rendered. It may
  /// also take a simulation step if the simulation is not paused.
  ///
  /// If you want to customize what happens at the beginning of each rendering
  /// cycle, you can either overload this function, or you can overload
  /// customUpdate(). This update() function will automatically call
  /// customUpdate() at the beginning of each rendering cycle. By default,
  /// customUpdate() does nothing.
  virtual void refresh();

  /// If update() is not overloaded, this function will be called at the
  /// beginning of each rendering cycle. This function can be overloaded to
  /// customize the behavior of each update. The default behavior is to do
  /// nothing, so overloading this function will not interfere with the usual
  /// update() operation.
  virtual void customPreRefresh();

  /// If update() is not overloaded, this function will be called at the end of
  /// each rendering cycle. This function can be overloaded to customize the
  /// behavior of each update. The default behavior is to do nothing, so
  /// overloading this function will not interfere with the usual update()
  /// operation.
  virtual void customPostRefresh();

  /// If update() is not overloaded, this function will be called at the
  /// beginning of each simulation step. This function can be overloaded to
  /// customize the behavior of each step. The default behavior is to do
  /// nothing, so overloading this function will not interfere with the usual
  /// update() operation. This will not get called if the simulation is paused.
  virtual void customPreStep();

  /// If update() is not overloaded, this function will be called at the end of
  /// each simulation step. This function can be overloaded to customize the
  /// behavior of each step. The default behavior is to do nothing, so
  /// overloading this function will not interfere with the usual update()
  /// operation. This will not get called if the simulation is paused.
  virtual void customPostStep();

  /// Returns true iff the WorldNode is stepping between render cycles
  bool isSimulating() const;

  /// Pass in true to take steps between render cycles; pass in false to turn
  /// off steps between render cycles.
  void simulate(bool _on);

  /// Set the number of steps to take between each render cycle (only if the
  /// simulation is not paused)
  void setNumStepsPerCycle(size_t _steps);

  /// Get the number of steps that will be taken between each render cycle (only
  /// if the simulation is not paused)
  size_t getNumStepsPerCycle() const;

protected:

  /// Destructor
  virtual ~WorldNode();

  /// Called when this world gets added to an osgKido::Viewer. Override this
  /// function to customize the way your WorldNode starts up in an
  /// osgKido::Viewer. Default behavior does nothing.
  virtual void setupViewer();

  /// Clear the utilization flags of each child node
  void clearChildUtilizationFlags();

  /// Clear any nodes whose utilization flags were not triggered on this render
  /// cycle
  void clearUnusedNodes();

  /// Refresh all the Skeleton rendering data
  void refreshSkeletons();

  /// Refresh all the custom Frame rendering data
  void refreshCustomFrames();

  /// Refresh the specified Frame's rendering data
  void refreshBaseFrameNode(kido::dynamics::Frame* _frame);

  /// Create a node for the specified Frame
  void createBaseFrameNode(kido::dynamics::Frame* _frame);

  /// Refresh the specified Entity's rendering data
  void refreshBaseEntityNode(kido::dynamics::Entity* _entity);

  /// Create a node for the specified Entity
  void createBaseEntityNode(kido::dynamics::Entity* _entity);

  /// Map from Frame pointers to child FrameNode pointers
  std::map<kido::dynamics::Frame*, FrameNode*> mFrameToNode;

  /// Map from child FrameNode pointers to Frame pointers
  std::map<FrameNode*, kido::dynamics::Frame*> mNodeToFrame;

  /// Map from Entity pointers to child EntityNode pointers
  std::map<kido::dynamics::Entity*, EntityNode*> mEntityToNode;

  /// Map from child EntityNode pointers to Entity pointers
  std::map<EntityNode*, kido::dynamics::Entity*> mNodeToEntity;

  /// The World that this WorldNode is associated with
  std::shared_ptr<kido::simulation::World> mWorld;

  /// True iff simulation is active
  bool mSimulating;

  /// Number of steps to take between rendering cycles
  size_t mNumStepsPerCycle;

  /// Viewer that this WorldNode is inside of
  Viewer* mViewer;

};

} // namespace osgKido

#endif // OSGKIDO_WORLDNODE_H
