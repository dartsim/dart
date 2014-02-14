/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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

#include "dart/utils/Paths.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/constraint/BallJointConstraint.h"

#include "apps/closedLoop/MyWindow.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace constraint;

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World* myWorld
            = utils::SkelParser::readSkelFile(DART_DATA_PATH"/skel/chain.skel");
    assert(myWorld != NULL);
    
    // create and initialize the world
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);
    myWorld->setTimeStep(1.0/2000);

    int dof =  myWorld->getSkeleton(0)->getNumGenCoords();

    Eigen::VectorXd initPose(dof);
    initPose.setZero();
    initPose[20] = 3.14159 * 0.4;
    initPose[23] = 3.14159 * 0.4;
    initPose[26] = 3.14159 * 0.4;
    initPose[29] = 3.14159 * 0.4;
    myWorld->getSkeleton(0)->setConfig(initPose);

    // create a ball joint constraint
    BodyNode *bd1 = myWorld->getSkeleton(0)->getBodyNode("link 6");
    BodyNode *bd2 = myWorld->getSkeleton(0)->getBodyNode("link 10");
    Eigen::Vector3d offset1 = bd1->getParentJoint()->getTransformFromChildBodyNode().translation();
    Eigen::Vector3d offset2(0.0, -0.025, 0.0);
    BallJointConstraint *cl = new BallJointConstraint(bd1, bd2, offset1, offset2);
    myWorld->getConstraintHandler()->addConstraint(cl);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Closed Loop");
    glutMainLoop();

    return 0;
}
