/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "apps/harnessTest/MyWindow.h"

#include "dart/utils/Paths.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"

using namespace dart;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World* myWorld
            = utils::SkelParser::readSkelFile(DART_DATA_PATH"skel/fullbody1.skel");
    assert(myWorld != NULL);

    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    myWorld->setGravity(gravity);

    std::vector<int> genCoordIds;
    genCoordIds.push_back(1);   // global orientation y
    genCoordIds.push_back(4);   // global position y
    genCoordIds.push_back(6);   // left hip
    genCoordIds.push_back(14);  // left knee
    genCoordIds.push_back(17);  // left ankle
    genCoordIds.push_back(9);   // right hip
    genCoordIds.push_back(15);  // right knee
    genCoordIds.push_back(19);  // right ankle
    genCoordIds.push_back(13);  // lower back
    Eigen::VectorXd initConfig(9);
    initConfig << -0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
    myWorld->getSkeleton(1)->setConfig(genCoordIds, initConfig);

    // create controller
    Controller* myController = new Controller(myWorld->getSkeleton(1),
                                              myWorld->getConstraintHandler(),
                                              myWorld->getTimeStep());

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;
    std::cout << "'h': harness on/off" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Harness Test");
    glutMainLoop();

    return 0;
}

