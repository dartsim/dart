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

#ifndef DART_GUI_OSG_WORLDNODE_HPP_
#define DART_GUI_OSG_WORLDNODE_HPP_

#include <osg/Group>
#include <osgShadow/ShadowTechnique>
#include <unordered_map>
#include <memory>

#include "dart/gui/osg/Viewer.hpp"

namespace dart {

namespace simulation {
class World;
} // namespace simulation

namespace dynamics {
class Frame;
class Entity;
class ShapeFrame;
} // namespace dynamics

namespace gui {
namespace osg {

class FrameNode;
class ShapeFrameNode;
class EntityNode;
class Viewer;

/// WorldNode class encapsulates a World to be displayed in OpenSceneGraph
class WorldNode : public ::osg::Group
{
public:

  friend class Viewer;

  /// Default constructor
  /// Shadows are disabled by default
  explicit WorldNode(std::shared_ptr<dart::simulation::World> world = nullptr, ::osg::ref_ptr<osgShadow::ShadowTechnique> shadowTechnique = nullptr);

  /// Set the World that this WorldNode is associated with
  void setWorld(std::shared_ptr<dart::simulation::World> newWorld);

  /// Get the World that this WorldNode is associated with
  std::shared_ptr<dart::simulation::World> getWorld() const;

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
  void simulate(bool on);

  /// Set the number of steps to take between each render cycle (only if the
  /// simulation is not paused)
  void setNumStepsPerCycle(std::size_t steps);

  /// Get the number of steps that will be taken between each render cycle (only
  /// if the simulation is not paused)
  std::size_t getNumStepsPerCycle() const;

  /// Get whether the WorldNode is casting shadows
  bool isShadowed() const;

  /// Set the ShadowTechnique
  /// If you wish to disable shadows, pass a nullptr
  void setShadowTechnique(::osg::ref_ptr<osgShadow::ShadowTechnique> shadowTechnique = nullptr);

  /// Get the current ShadowTechnique
  /// nullptr is there are no shadows
  ::osg::ref_ptr<osgShadow::ShadowTechnique> getShadowTechnique() const;

  /// Helper function to create a default ShadowTechnique given a Viewer
  /// the default ShadowTechnique is ShadowMap
  static ::osg::ref_ptr<osgShadow::ShadowTechnique> createDefaultShadowTechnique(const Viewer* viewer);

protected:

  /// Destructor
  virtual ~WorldNode();

  /// Called when this world gets added to an dart::gui::osg::Viewer. Override
  /// this function to customize the way your WorldNode starts up in an
  /// dart::gui::osg::Viewer. Default behavior does nothing.
  virtual void setupViewer();

  /// Clear the utilization flags of each child node
  void clearChildUtilizationFlags();

  /// Clear any nodes whose utilization flags were not triggered on this render
  /// cycle
  void clearUnusedNodes();

  /// Refresh all the Skeleton rendering data
  void refreshSkeletons();

  /// Refresh all the custom Frame rendering data
  void refreshSimpleFrames();

  void refreshBaseFrameNode(dart::dynamics::Frame* frame);

  void refreshShapeFrameNode(dart::dynamics::Frame* frame);

  using NodeMap = std::unordered_map<dart::dynamics::Frame*, ShapeFrameNode*>;

  /// Map from Frame pointers to FrameNode pointers
  NodeMap mFrameToNode;

  /// The World that this WorldNode is associated with
  std::shared_ptr<dart::simulation::World> mWorld;

  /// True iff simulation is active
  bool mSimulating;

  /// Number of steps to take between rendering cycles
  std::size_t mNumStepsPerCycle;

  /// Viewer that this WorldNode is inside of
  Viewer* mViewer;

  /// OSG group for non-shadowed objects
  ::osg::ref_ptr<::osg::Group> mNormalGroup;

  /// OSG group for shadowed objects
  ::osg::ref_ptr<::osg::Group> mShadowedGroup;

  /// Whether the shadows are enabled
  bool mShadowed;

};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_WORLDNODE_HPP_
