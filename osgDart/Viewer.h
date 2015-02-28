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

#ifndef OSGDART_VIEWER_H
#define OSGDART_VIEWER_H

#include <map>

#include <osgViewer/Viewer>

#include <Eigen/Core>

namespace dart {
namespace simulation {
class World;
} // namespace simulation
} // namespace dart

namespace osgDart
{

class WorldNode;
class DefaultEventHandler;

class Viewer : public osgViewer::Viewer
{
public:

  /// Constructor for osgDart::Viewer. This will automatically create the
  /// default event handler.
  Viewer(const osg::Vec4& clearColor = osg::Vec4(0.9,0.9,0.9,1.0));

  /// Destructor
  virtual ~Viewer();

  /// Creates the default event handler for this osgDart::Viewer
  virtual void switchDefaultEventHandler(bool _on);

  /// Return a pointer to the default event handler
  DefaultEventHandler* getDefaultEventHandler() const;

  /// Pass in true to turn headlights on, false to turn headlights off
  virtual void switchHeadlights(bool _on);

  /// True iff headlights are currently set to true
  bool checkHeadlights() const;

  /// Add a WorldNode to this Viewer. If active is true, the WorldNode will
  /// respond to user input (but it will not begin simulating unless its
  /// simulation is currently turned on).
  void addWorldNode(WorldNode* _newWorldNode, bool _active=true);

  /// Remove a WorldNode from this Viewer
  void removeWorldNode(WorldNode* _oldWorldNode);

  /// Remove the WorldNode associated with _oldWorld from this Viewer
  void removeWorldNode(dart::simulation::World* _oldWorld);

  /// Get the WorldNode associated with the given _world. Returns nullptr if
  /// this Viewer does not contain a WorldNode associated with _world.
  WorldNode* getWorldNode(dart::simulation::World* _world) const;

  /// Get the Group node that contains the LightSources for this Viewer
  osg::Group* getLightGroup();

  /// Get the Group node that contains the LightSources for this Viewer
  const osg::Group* getLightGroup() const;

  /// Set up the default lighting scheme
  void setupDefaultLights();

  /// Set the direction that this Viewer should consider to be upwards (default
  /// is <0,0,1>)
  void setUpwardsDirection(const osg::Vec3& _up);

  /// Set the direction that this Viewer should consider to be upwards (default
  /// is <0,0,1>)
  void setUpwardsDirection(const Eigen::Vector3d& _up);

  /// Set the given WorldNode to active
  void setWorldNodeActive(WorldNode* _node, bool _active=true);

  /// Set the given World to active
  void setWorldNodeActive(dart::simulation::World* _world, bool _active=true);

  /// Set all currently active WorldNodes to simulate _on
  void simulate(bool _on);

  /// Returns true iff this Viewer is currently set to simulate
  bool isSimulating() const;

  /// Get a string containing the user interface constructions for this Viewer
  const std::string& getInstructions() const;

  /// Add something to the instructions for this Viewer. You are strongly
  /// recommended to end your string with an end-of-line character. An
  /// end-of-line character will NOT be added automatically.
  void addInstructionText(const std::string& _instruction);

protected:

  /// Default WorldNodeEventHandler for this osgDart::Viewer
  osg::ref_ptr<DefaultEventHandler> mDefaultEventHandler;

  /// The root node of this Viewer
  osg::ref_ptr<osg::Group> mRootGroup;

  /// The Group Node containing light sources
  osg::ref_ptr<osg::Group> mLightGroup;

  /// Non-headlights Light #1
  osg::ref_ptr<osg::Light> mLight1;

  /// Non-headlights LightSource #1
  osg::ref_ptr<osg::LightSource> mLightSource1;

  /// Non-headlights Light #2
  osg::ref_ptr<osg::Light> mLight2;

  /// Non-headlights LightSource #2
  osg::ref_ptr<osg::LightSource> mLightSource2;

  /// Vector pointing upwards
  osg::Vec3 mUpwards;

  /// Vector pointing to the side
  osg::Vec3 mOver;

  /// True iff this Viewer is currently simulating
  bool mSimulating;

  /// True iff headlights were last set to be on
  bool mHeadlights;

  /// Map of WorldNodes in this osgDart::Viewer. A WorldNode will map to true
  /// iff it is currently active
  std::map<WorldNode*,bool> mWorldNodes;

  /// string of instructions for this Viewer
  std::string mInstructions;

};

}

#endif // OSGDART_VIEWER_H
