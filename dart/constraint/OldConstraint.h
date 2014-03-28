/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu
 * Date:
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
 */

#ifndef DART_CONSTRAINT_CONSTRAINT_H
#define DART_CONSTRAINT_CONSTRAINT_H

#include <vector>
#include <Eigen/Dense>

namespace dart {

namespace dynamics {
class BodyNode;
}

namespace constraint {

class Constraint {
public:
  Constraint() {}
  virtual ~Constraint() {}

  virtual void updateDynamics(Eigen::MatrixXd & _J1, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {}
  virtual void updateDynamics(Eigen::MatrixXd & _J1, Eigen::MatrixXd & _J2, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {}
  inline int getNumRows() const { return mNumRows; }
  inline Eigen::VectorXd getLagrangeMultipliers() const { return mLagrangeMultipliers; }
  inline void setLagrangeMultipliers(const Eigen::VectorXd& _lambda) { mLagrangeMultipliers = _lambda; }
  inline dynamics::BodyNode* getBodyNode1() { return mBodyNode1; }
  inline dynamics::BodyNode* getBodyNode2() { return mBodyNode2; }

protected:
  virtual void getJacobian() {}
  int mNumRows;
  Eigen::VectorXd mLagrangeMultipliers;

  dynamics::BodyNode* mBodyNode1;
  dynamics::BodyNode* mBodyNode2;
  Eigen::MatrixXd mJ1;
  Eigen::MatrixXd mJ2;
};

} // namespace constraint
} // namespace dart

#endif // #ifndef DART_CONSTRAINT_CONSTRAINT_H

