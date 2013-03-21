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
#include <dynamics/ContactDynamics.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/Robot.h>

using namespace Eigen;

namespace robotics {

  /**
   * @function World
   * @brief Constructor
   */
  World::World() :
    mTime(0.0),
    mTimeStep(0.001)
  {
    mRobots.resize(0);
    mObjects.resize(0);
    mSkeletons.resize(0);
    mIndices.push_back(0);

    mGravity = Eigen::Vector3d(0, 0, -9.81);
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
  
  void World::rebuildCollision()
  {
    // for(unsigned int i = 0; i < mSkeletons.size(); i++)
    //   mSkeletons[i]->initDynamics();
    mCollisionHandle = new dynamics::ContactDynamics(mSkeletons, mTimeStep);
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
    
    mIndices.push_back(mIndices.back() + _robot->getNumDofs());

    if(!_robot->getImmobileState()) {
      _robot->computeDynamics(mGravity, _robot->getQDotVector(), false); // Not sure if we need this
    }

    // create collision dynamics object
    // rebuildCollision();

    assert(mObjects.size() + mRobots.size() == mSkeletons.size());

    return mRobots.size();
  }

  /**
   * @function addObject
   * @brief Add a pointer to a new object in the World
   */
  int World::addObject( Robot* _object ) {
    // add item
    mObjects.push_back( _object );
    mSkeletons.push_back( _object );

    _object->initDynamics();

    mIndices.push_back(mIndices.back() + _object->getNumDofs());

    if(!_object->getImmobileState()) {
      _object->computeDynamics(mGravity, _object->getQDotVector(), false); // Not sure if we need this
    }

    // create collision dynanmics object
    // rebuildCollision();

    assert(mObjects.size() + mRobots.size() == mSkeletons.size());
    
    return mObjects.size();
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
  Robot* World::getObject( int _i ) {
    return mObjects[_i];
  }
  
  /**
   * @function getRobot
   */
  Robot* World::getRobot( int _i ) {
    return mRobots[_i];
  }

  /**
   * @function getSkeleton
   */
  dynamics::SkeletonDynamics* World::getSkeleton( int _i ) {
    return mSkeletons[_i];
  }

  dynamics::SkeletonDynamics* World::getSkeleton(const std::string& _name) {\
    for(unsigned int i = 0; i < mSkeletons.size(); i++) {
      if(mSkeletons[i]->getName().compare(_name) == 0) {
        return mSkeletons[i];
      }
    }
    return NULL;
  }
  
  bool World::checkCollision(bool checkAllCollisions) {
    return mCollisionHandle->getCollisionChecker()->checkCollision(checkAllCollisions, false);
  }

  void World::step() {
    mIntegrator.integrate(this, mTimeStep);
    mTime += mTimeStep;
  }

  VectorXd World::getState() {
    VectorXd state(mIndices.back() * 2);
    for (int i = 0; i < getNumSkeletons(); i++) {
      int start = mIndices[i] * 2;
      int size = getSkeleton(i)->getNumDofs();
      state.segment(start, size) = getSkeleton(i)->getPose();
      state.segment(start + size, size) = getSkeleton(i)->getQDotVector();
    }
    return state;
  }

  VectorXd World::evalDeriv() {
    // compute contact forces
    mCollisionHandle->applyContactForces();

    // compute derivatives for integration
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);
    for (int i = 0; i < getNumSkeletons(); i++) {
        // skip immobile objects in forward simulation
        if (getSkeleton(i)->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumDofs();
        VectorXd qddot = getSkeleton(i)->getMassMatrix().ldlt().solve(
            - getSkeleton(i)->getCombinedVector() + getSkeleton(i)->getExternalForces()
            + mCollisionHandle->getConstraintForce(i) + getSkeleton(i)->getInternalForces());

        deriv.segment(start, size) = getSkeleton(i)->getQDotVector() + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    }
    return deriv;
}

void World::setState(const VectorXd &newState) {
    for (int i = 0; i < getNumSkeletons(); i++) {
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumDofs();

        VectorXd pose = newState.segment(start, size);
        VectorXd qDot = newState.segment(start + size, size);
        getSkeleton(i)->clampRotation(pose, qDot);

        if (getSkeleton(i)->getImmobileState()) {
            // need to update node transformation for collision
            getSkeleton(i)->setPose(pose, true, false);
        } else {
            // need to update first derivatives for collision
            getSkeleton(i)->setPose(pose, false, false);
            getSkeleton(i)->computeDynamics(mGravity, qDot, true);
        }
    }
}

} // end namespace robotics


// Local Variables:
// c-basic-offset: 2
// End:
