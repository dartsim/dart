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

#include <dart/dart.hpp>
#include <dart/io/io.hpp>

#include "MyWindow.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace constraint;

int main(int argc, char* argv[])
{
  // load a skeleton file
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
    = io::SkelParser::readWorld("dart://sample/skel/chain.skel");
  assert(myWorld != nullptr);
    
  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);
  myWorld->setTimeStep(1.0/2000);

  int dof =  myWorld->getSkeleton(0)->getNumDofs();

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
  bd1->getShapeNode(0)->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  bd2->getShapeNode(0)->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  Eigen::Vector3d offset(0.0, 0.025, 0.0);
  Eigen::Vector3d jointPos = bd1->getTransform() * offset;
  BallJointConstraintPtr cl =
      std::make_shared<BallJointConstraint>( bd1, bd2, jointPos);
  //WeldJointConstraint *cl = new WeldJointConstraint(bd1, bd2);
  myWorld->getConstraintSolver()->addConstraint(cl);

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
  
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Closed Loop");
  glutMainLoop();

  return 0;
}
