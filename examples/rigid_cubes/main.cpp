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

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::osg;
using namespace dart::utils;
using namespace dart::math;

const int default_playback_frame_step = 16;

class RigidCubesWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  RigidCubesWorldNode(
      dart::simulation::WorldPtr world,
      ::osg::ref_ptr<osgShadow::ShadowTechnique> shadow = nullptr)
    : dart::gui::osg::RealTimeWorldNode(std::move(world), std::move(shadow)),
      mForce(Eigen::Vector3d::Zero()),
      mPlayingBack(false),
      mPlayFrame(0)
  {
  }

  void customPreRefresh() override
  {
    if (!mPlayingBack)
      return;

    Recording* recording = mWorld->getRecording();
    const int numFrames = recording->getNumFrames();
    if (numFrames == 0) {
      stopPlayback();
      return;
    }

    if (recording->getNumSkeletons()
        != static_cast<int>(mWorld->getNumSkeletons())) {
      stopPlayback();
      return;
    }

    if (mPlayFrame >= numFrames)
      mPlayFrame = 0;

    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
      mWorld->getSkeleton(i)->setPositions(recording->getConfig(mPlayFrame, i));

    mPlayFrame += default_playback_frame_step;
  }

  void customPreStep() override
  {
    // Apply the force to the second skeleton (index 1), first body node
    if (mWorld->getNumSkeletons() > 1) {
      mWorld->getSkeleton(1)->getBodyNode(0)->addExtForce(mForce);
    }

    // Decay the force by half each step (as in original)
    mForce /= 2.0;
  }

  void customPostStep() override
  {
    mWorld->bake();
  }

  void setForce(const Eigen::Vector3d& force)
  {
    mForce = force;
  }

  void togglePlayback()
  {
    Recording* recording = mWorld->getRecording();
    if (recording->getNumFrames() == 0) {
      std::cout << "No recorded frames are available for playback.\n";
      return;
    }

    mPlayingBack = !mPlayingBack;
    if (mPlayingBack && mViewer)
      mViewer->simulate(false);

    if (mPlayingBack && mPlayFrame >= recording->getNumFrames())
      mPlayFrame = 0;
  }

  void stopPlayback()
  {
    mPlayingBack = false;
  }

protected:
  Eigen::Vector3d mForce;
  bool mPlayingBack;
  int mPlayFrame;
};

class RigidCubesEventHandler : public ::osgGA::GUIEventHandler
{
public:
  RigidCubesEventHandler(
      dart::gui::osg::Viewer* viewer, RigidCubesWorldNode* worldNode)
    : mViewer(viewer), mWorldNode(worldNode)
  {
  }

  virtual bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType()) {
      switch (ea.getKey()) {
        case ' ': // Space bar: toggle simulation
          if (mWorldNode)
            mWorldNode->stopPlayback();
          return false;
        case 'p': // Toggle playback mode
          if (mWorldNode)
            mWorldNode->togglePlayback();
          return true;
        case 'v': // Toggle visualization markers
          if (mWorldNode) {
            // Toggle marker visibility (implementation would depend on specific
            // markers)
            std::cout << "Toggling visualization markers\n";
          }
          return true;
        case '1': // Apply negative X force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(-500, 0, 0));
            std::cout << "Applied -X force\n";
          }
          return true;
        case '2': // Apply positive X force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(500, 0, 0));
            std::cout << "Applied +X force\n";
          }
          return true;
        case '3': // Apply negative Z force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(0, 0, -500));
            std::cout << "Applied -Z force\n";
          }
          return true;
        case '4': // Apply positive Z force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(0, 0, 500));
            std::cout << "Applied +Z force\n";
          }
          return true;
        default:
          return false;
      }
    }
    return false;
  }

protected:
  dart::gui::osg::Viewer* mViewer;
  RigidCubesWorldNode* mWorldNode;
};

int main()
{
  // Create and initialize the world
  auto world
      = dart::utils::SkelParser::readWorld("dart://sample/skel/cubes.skel");
  if (!world) {
    dterr << "Failed to load world.\n";
    return EXIT_FAILURE;
  }
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  // Create OSG viewer
  dart::gui::osg::Viewer viewer;

  // Create shadow technique
  auto shadow
      = dart::gui::osg::WorldNode::createDefaultShadowTechnique(&viewer);

  // Create custom world node
  ::osg::ref_ptr<RigidCubesWorldNode> worldNode
      = new RigidCubesWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  // Create and add event handler
  ::osg::ref_ptr<RigidCubesEventHandler> eventHandler
      = new RigidCubesEventHandler(&viewer, worldNode);
  viewer.addEventHandler(eventHandler);

  // Set up the viewer window
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Set up camera position
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 5.0f, 5.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Add instruction text
  viewer.addInstructionText("Rigid Cubes Example\n");
  viewer.addInstructionText("Controls:\n");
  viewer.addInstructionText("  space bar: simulation on/off\n");
  viewer.addInstructionText("  'p': playback/stop\n");
  viewer.addInstructionText("  'v': visualization on/off\n");
  viewer.addInstructionText("  '1'-'4': apply directional forces\n");
  viewer.addInstructionText("    '1': -X force    '2': +X force\n");
  viewer.addInstructionText("    '3': -Z force    '4': +Z force\n");

  // Print instructions to console
  std::cout << viewer.getInstructions() << std::endl;

  // Run the simulation
  return viewer.run();
}
