/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 10/7/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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
 **
 * @file World.cpp
 */

#include <iostream>
#include "World.h"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"

using namespace std;
using namespace Eigen;

namespace planning {

    /**
     * @function World
     * @brief Constructor
     */
    World::World() {
        mRobots.resize(0); mObjects.resize(0);
    }

  
    /**
     * @function ~World
     * @brief Destructor
     */
    World::~World() {
        for( unsigned int i = 0; i < mRobots.size(); i++ ) delete mRobots[i];
        mRobots.clear();
        for( unsigned int i = 0; i < mObjects.size(); i++ ) delete mObjects[i];
        mObjects.clear();      
    }

    /**
     * @function addRobot
     */
    int World::addRobot( Robot *_robot ) {
        mRobots.push_back( _robot );
        return mRobots.size();
    }

    /**
     * @function addObject
     */
    int World::addObject( Object *_object ) {
        mObjects.push_back( _object );
        return mObjects.size();
    } 

    /**
     * @function checkCollisions
     * @brief True if there is collisions False if there is NOT collisions whatsoever
     */
    bool World::checkCollisions() {
        return false;
    }

    /**
     * @function printInfo
     */
    void World::printInfo() {

        /// Robots
        std::cout<<"-- World has " << mRobots.size() << " robots "<< std::endl;
        for( unsigned int i = 0; i < mRobots.size(); i++ )
        {    std::cout<<"* Robot["<<i<<"]: " << mRobots[i]->mName << " with "<< mRobots[i]->getNumQuickDofs() << " DOFs:" << std::endl; }

        /// Objects
        std::cout<<"-- World has " << mObjects.size() << " objects "<< std::endl;
        for( unsigned int i = 0; i < mObjects.size(); i++ )
        { std::cout<<"* Object["<<i<<"]: " << mObjects[i]->mName << std::endl; }

    }   

}  // namespace planning
