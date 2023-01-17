/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef EXAMPLES_JOINTCONSTRAINTS_CONTROLLER_HPP_
#define EXAMPLES_JOINTCONSTRAINTS_CONTROLLER_HPP_

#include <dart/dart.hpp>

#include <Eigen/Dense>

#include <vector>

class Controller
{
public:
  Controller(
      const dart::dynamics::SkeletonPtr& _skel,
      dart::dynamics::ConstraintSolver* _collisionSolver,
      double _t);
  virtual ~Controller() {}

  dart::math::VectorXd getTorques()
  {
    return mTorques;
  }
  double getTorque(int _index)
  {
    return mTorques[_index];
  }
  void setDesiredDof(int _index, double _val)
  {
    mDesiredDofs[_index] = _val;
  }
  void computeTorques(
      const dart::math::VectorXd& _dof, const dart::math::VectorXd& _dofVel);
  dart::dynamics::SkeletonPtr getSkel()
  {
    return mSkel;
  }
  dart::math::VectorXd getDesiredDofs()
  {
    return mDesiredDofs;
  }
  dart::math::MatrixXd getKp()
  {
    return mKp;
  }
  dart::math::MatrixXd getKd()
  {
    return mKd;
  }
  void setConstrForces(const dart::math::VectorXd& _constrForce)
  {
    mConstrForces = _constrForce;
  }

protected:
  bool computeCoP(dart::dynamics::BodyNode* _node, dart::math::Vector3d* _cop);
  dart::math::Vector3d evalLinMomentum(const dart::math::VectorXd& _dofVel);
  dart::math::Vector3d evalAngMomentum(const dart::math::VectorXd& _dofVel);
  dart::math::VectorXd adjustAngMomentum(
      dart::math::VectorXd _deltaMomentum,
      dart::math::VectorXd _controlledAxis);
  dart::dynamics::SkeletonPtr mSkel;
  dart::dynamics::ConstraintSolver* mCollisionHandle;
  dart::math::VectorXd mTorques;
  dart::math::VectorXd mDesiredDofs;
  dart::math::MatrixXd mKp;
  dart::math::MatrixXd mKd;
  int mFrame;
  double mTimestep;
  double mPreOffset;
  dart::math::VectorXd
      mConstrForces; // SPD utilizes the current info about contact forces
};

#endif // EXAMPLES_JOINTCONSTRAINTS_CONTROLLER_HPP_
