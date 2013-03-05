/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana Huaman <ahuaman3@gatech.edu>
 * Date: 07/08/2012
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
 *   @file World.h
 *   @brief Class that contains robot + object descriptions
 */

#ifndef __DART_ROBOTICS_World_H__
#define __DART_ROBOTICS_World_H__

#include <vector>
#include <Eigen/Core>
#include "integration/EulerIntegrator.h"
#include "utils/Deprecated.h"

namespace dynamics {
  class SkeletonDynamics;
  class ContactDynamics;
}

namespace robotics {

  class Robot;

  /**
   * @class World
   */
  class World : public integration::IntegrableSystem {
      
  public:
    World();
    virtual ~World();
    
    int addRobot( Robot* _robot );
    int addObject( Robot* _object );
    void printInfo();
    
    DEPRECATED inline unsigned int getNumObjects() { return mObjects.size(); }
    DEPRECATED inline unsigned int getNumRobots() { return mRobots.size(); }
    inline unsigned int getNumSkeletons() { return mSkeletons.size(); }
    DEPRECATED Robot* getObject( int _i );
    DEPRECATED Robot* getRobot( int _i );
    dynamics::SkeletonDynamics* getSkeleton( int _i );
    dynamics::SkeletonDynamics* getSkeleton(const std::string& name);
    bool checkCollision(bool checkAllCollisions = false);
    void step();

    /* TODO: figure out a way for this to not be public */
    dynamics::ContactDynamics* mCollisionHandle;

    Eigen::Vector3d mGravity;   /* meters per second */
    const double mTimeStep; /* in seconds */
    double mTime;

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);

    void rebuildCollision();

  private:
    std::vector<Robot*> mRobots;
    std::vector<Robot*> mObjects;
    std::vector<dynamics::SkeletonDynamics*> mSkeletons;
    integration::EulerIntegrator mIntegrator;
    std::vector<int> mIndices;
  };

} // namespace robotics


#endif /** __DART_ROBOTICS_World_H__ */


