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
 * @file Environment.cpp
 */

#include <iostream>
#include "Environment.h"

namespace robotics {

  /**
   * @function Environment
   * @brief Constructor
   */
  Environment::Environment() {
    robots_.resize(0);
    objects_.resize(0);
  }

  /**
   * @function ~Environment
   * @brief Destructor
   */
  Environment::~Environment() {
    for( size_t i = 0; i < robots_.size(); ++i ) {
      delete robots_[i];
    }
    robots_.clear();

    for( size_t i = 0; i < objects_.size(); ++i ) {
      delete objects_[i];
    }
    objects_.clear();
  }
  
  /**
   * @function addRobot
   * @brief Add a pointer to a new robot in the environment
   */
  int Environment::addRobot( Robot* _robot ) {
    robots_.push_back( _robot );
    return robots_.size();
  }

  /**
   * @function addObject
   * @brief Add a pointer to a new object in the environment
   */
  int Environment::addObject( Object* _object ) {
    objects_.push_back( _object );
    return objects_.size();
  }

  /**
   * @function printStatus
   * @brief Print info w.r.t. robots and objects in Environment
   */
  void Environment::printStatus() {

    std::cout << "*  Environment Info * " << std::endl;
    std::cout << "----------------------" << std::endl;

    //-- Robots
    for( size_t i = 0; i < robots_.size(); ++i ) {
      std::cout << "* Robot["<<i<<"]: "<< robots_[i]->getName()<<" : "<< robots_[i]->getNumDofs() <<" DOFs " << std::endl;
    }
    //-- Objects
    for( size_t i = 0; i < objects_.size(); ++i ) {
      std::cout << "* Object["<<i<<"]: "<< objects_[i]->getName() << std::endl;
    }

  }

} // end namespace robotics

