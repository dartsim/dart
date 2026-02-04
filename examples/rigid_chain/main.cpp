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

#include "dart/common/macros.hpp"

#include <dart/gui/all.hpp>

#include <dart/utils/All.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

class RigidChainWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  RigidChainWorldNode(dart::simulation::WorldPtr world)
    : dart::gui::RealTimeWorldNode(world)
  {
  }

protected:
  void customPreStep() override
  {
    // Apply damping forces before each simulation step
    Eigen::VectorXd damping = computeDamping();
    mWorld->getSkeleton(0)->setForces(damping);
  }

private:
  Eigen::VectorXd computeDamping()
  {
    int nDof = mWorld->getSkeleton(0)->getNumDofs();
    // add damping to each joint; twist-dof has smaller damping
    Eigen::VectorXd damping = -0.01 * mWorld->getSkeleton(0)->getVelocities();
    for (int i = 0; i < nDof; i++) {
      if (i % 3 == 1) {
        damping[i] *= 0.1;
      }
    }
    return damping;
  }
};

int main()
{
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::io::readWorld("dart://sample/skel/chain.skel");
  DART_ASSERT(myWorld != nullptr);

  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);
  myWorld->setTimeStep(1.0 / 2000);

  int dof = myWorld->getSkeleton(0)->getNumDofs();
  Eigen::VectorXd initPose(dof);
  for (int i = 0; i < dof; i++) {
    initPose[i] = dart::math::Random::uniform(-0.5, 0.5);
  }
  myWorld->getSkeleton(0)->setPositions(initPose);

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<RigidChainWorldNode> node = new RigidChainWorldNode(myWorld);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::Viewer viewer;
  viewer.addWorldNode(node);

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 1.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
