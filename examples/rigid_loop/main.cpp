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

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace constraint;

class RigidLoopWorldNode : public dart::gui::WorldNode
{
public:
  RigidLoopWorldNode(dart::simulation::WorldPtr world)
    : dart::gui::WorldNode(world), mWorld(world)
  {
  }

  void customPreStep() override
  {
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

  dart::simulation::WorldPtr mWorld;
};

int main()
{
  // load a skeleton file
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
  initPose.setZero();
  initPose[20] = 3.14159 * 0.4;
  initPose[23] = 3.14159 * 0.4;
  initPose[26] = 3.14159 * 0.4;
  initPose[29] = 3.14159 * 0.4;
  myWorld->getSkeleton(0)->setPositions(initPose);

  // create a ball joint constraint
  BodyNode* bd1 = myWorld->getSkeleton(0)->getBodyNode("link 6");
  BodyNode* bd2 = myWorld->getSkeleton(0)->getBodyNode("link 10");
  bd1->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  bd2->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  Eigen::Vector3d offset(0.0, 0.025, 0.0);
  Eigen::Vector3d jointPos = bd1->getTransform() * offset;
  BallJointConstraintPtr cl
      = std::make_shared<BallJointConstraint>(bd1, bd2, jointPos);
  // WeldJointConstraint *cl = new WeldJointConstraint(bd1, bd2);
  myWorld->getConstraintSolver()->addConstraint(cl);

  // Create OSG viewer
  dart::gui::Viewer viewer;
  viewer.addWorldNode(new RigidLoopWorldNode(myWorld));

  // Print instructions
  std::cout << "Rigid Loop Chain Simulation\n";
  std::cout << "Red links are connected by a ball joint constraint to form a "
               "closed loop\n";
  std::cout << "Space bar: Play/pause simulation\n";
  std::cout << "ESC: Exit\n" << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Run the viewer
  viewer.run();

  return 0;
}
