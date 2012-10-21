/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 03-08-2012
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
#include <kinematics/BodyNode.h>
#include <kinematics/Shape.h>
#include <collision/CollisionSkeleton.h>
#include <integration/EulerIntegrator.h>

namespace robotics {

  /**
   * @function World
   * @brief Constructor
   */
  World::World() {
    mRobots.resize(0);
    mObjects.resize(0);
    mSkeletons.resize(0);

    mTimeStep = .01;
  }

  /**
   * @function ~World
   * @brief Destructor
   */
  World::~World() {
    for( size_t i = 0; i < mRobots.size(); ++i ) {
      delete mRobots[i];
    }
    mRobots.clear();

    for( size_t i = 0; i < mObjects.size(); ++i ) {
      delete mObjects[i];
    }
    mObjects.clear();

    mSkeletons.clear();
  }
  
  /**
   * @function addRobot
   * @brief Add a pointer to a new robot in the World
   */
  int World::addRobot( Robot* _robot ) {
    // add item
    mRobots.push_back( _robot );
    mSkeletons.push_back( _robot );

    _robot->initDynamics();
    
    // recreate collision dynamics object
    if (mCollisionHandle)
      delete mCollisionHandle;
    mCollisionHandle = new dynamics::ContactDynamics(mSkeletons, mTimeStep);

    return mRobots.size();
  }

  /**
   * @function addObject
   * @brief Add a pointer to a new object in the World
   */
  int World::addObject( Object* _object ) {
    std::cout << "DEBUG: adding object: " << _object->getName() << std::endl;
    
    // add item
    mObjects.push_back( _object );
    mSkeletons.push_back( _object );

    std::cout << "DEBUG: pushed back object: " << _object->getName() << std::endl;

    _object->initDynamics();

    std::cout << "DEBUG: init dynamics: " << _object->getName() << std::endl;

    // recreate collision dynamics object
    if (mCollisionHandle)
      delete mCollisionHandle;
    mCollisionHandle = new dynamics::ContactDynamics(mSkeletons, mTimeStep);
    
    return mObjects.size();
  }

  int World::totalNumDofs()
  {
    int result = 0;
    for( size_t i = 0; i < mRobots.size(); ++i ) {
      result += mRobots[i]->getNumDofs();
    }
    //-- Objects
    for( size_t i = 0; i < mObjects.size(); ++i ) {
      result += mObjects[i]->getNumDofs();
    }
  }

  // state is vector of doubles
  // altnerating position and velocity for every dof in the world
  // organized straight out of mskeletons
  Eigen::VectorXd World::getState()
  {
    int curStateIndex = 0;
    Eigen::VectorXd state = Eigen::VectorXd(5);
    return state;
  }

  Eigen::VectorXd World::evalDeriv()
  {
    Eigen::VectorXd state = Eigen::VectorXd(5);
    return state;
  }

  void World::setState(Eigen::VectorXd state)
  {
  }
  
  /**
   * @function printInfo
   * @brief Print info w.r.t. robots and objects in World
   */
  void World::printInfo() {

    std::cout << "*  World Info * " << std::endl;
    std::cout << "----------------------" << std::endl;

    //-- Robots
    for( size_t i = 0; i < mRobots.size(); ++i ) {
      std::cout << "* Robot["<<i<<"]: "<< mRobots[i]->getName()<<" : "<< mRobots[i]->getNumDofs() <<" DOFs " << std::endl;
    }
    //-- Objects
    for( size_t i = 0; i < mObjects.size(); ++i ) {
      std::cout << "* Object["<<i<<"]: "<< mObjects[i]->getName() << std::endl;
    }

  }

  /**
   * @function getObject
   */
  Object* World::getObject( int _i ) {
    return mObjects[_i];
  }
  
  /**
   * @function getRobot
   */
  Robot* World::getRobot( int _i ) {
    return mRobots[_i];
  }

  bool World::checkCollision() {
    return mCollisionHandle->getCollisionChecker()->checkCollision(false);
  }

} // end namespace robotics


// Local Variables:
// c-basic-offset: 2
// End:
