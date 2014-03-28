/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date:
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

#ifndef DART_CONSTRAINT_CONSTRAINTDYNAMICS_H_
#define DART_CONSTRAINT_CONSTRAINTDYNAMICS_H_

#include <vector>

#include <Eigen/Dense>

#include "dart/constraint/Constraint.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"

#define DART_DEFAULT_CONTACT_ERP     0.2  // should be in range of [0.0, 1.0]
#define DART_DEFAULT_JOINT_LIMIT_ERP 0.2  // should be in range of [0.0, 1.0]

#define DART_DEFAULT_MAX_CONTACT_ERV     10.0
#define DART_DEFAULT_MAX_JOINT_LIMIT_ERV 10.0

#define DART_DEFAULT_CONTACT_ERROR_ALLOWANCE     0.00
#define DART_DEFAULT_JOINT_LIMIT_ERROR_ALLOWANCE 0.00

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
class BodyNode;
}  // namespace dynamics
}  // namespace dart

namespace dart {
namespace constraint {

class ConstraintDynamics
{
public:
  ConstraintDynamics(const std::vector<dynamics::Skeleton*>& _skels,
                     double _dt, double _mu = 1.0, int _d = 4,
                     bool _useODE = true,
                     collision::CollisionDetector* _collisionDetector =
                       new collision::FCLMeshCollisionDetector());
  virtual ~ConstraintDynamics();

  void computeConstraintForces();
  void addConstraint(Constraint *_constr);
  void deleteConstraint(Constraint *_constr);
  void deleteConstraint();
  void addSkeleton(dynamics::Skeleton* _skeleton);
  void removeSkeleton(dynamics::Skeleton* _skeleton);
  void setTimeStep(double _timeStep);
  double getTimeStep() const;
  void setCollisionDetector(collision::CollisionDetector* _collisionDetector);

  Eigen::VectorXd getTotalConstraintForce(int _skelIndex) const;
  Eigen::VectorXd getContactForce(int _skelIndex) const;
  collision::CollisionDetector* getCollisionDetector() const;
  int getNumContacts() const;
  Constraint* getConstraint(int _index) const;

  void setContactErrorAllowance(double _allowance);
  double getContactErrorAllowance() const;
  void setContactERP(double _erp);
  double getContactERP() const;
  void setMaxContactERV(double _maxErv);
  double getMaxContactERV() const;

  void setJointLimitErrorAllowance(double _allowance);
  double getJointLimitErrorAllowance() const;
  void setJointLimitERP(double _erp);
  double getJointLimitERP() const;
  void setMaxJointLimitERV(double _maxErv);
  double getMaxJointLimitERV() const;

protected:
  void initialize();

  void computeConstraintWithoutContact();
  virtual void fillMatrices();
  virtual void fillMatricesODE();
  bool solve();
  virtual void applySolution();
  virtual void applySolutionODE();

  void updateMassMat();
  void updateTauStar();
  virtual void updateNBMatrices();
  virtual void updateNBMatricesODE();
  virtual Eigen::MatrixXd getJacobian(dynamics::BodyNode* node,
                                      const Eigen::Vector3d& p);
  // gets a matrix of tangent dirs.
  Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& n);
  // gets a matrix of tangent dirs.
  Eigen::MatrixXd getTangentBasisMatrixODE(const Eigen::Vector3d& p,
                                           const Eigen::Vector3d& n);
  Eigen::MatrixXd getContactMatrix() const;  // E matrix
  Eigen::MatrixXd getMuMatrix() const;  // mu matrix
  void updateConstraintTerms();

  int getTotalNumDofs() const;

  std::vector<dynamics::Skeleton*> mSkeletons;
  std::vector<int> mBodyIndexToSkelIndex;
  std::vector<int> mIndices;
  collision::CollisionDetector* mCollisionDetector;
  double mDt;  // timestep
  double mMu;  // friction coeff.
  int mNumDir;  // number of basis directions

  // Cached (aggregated) mass/tau matrices
  Eigen::MatrixXd mMInv;
  Eigen::VectorXd mTauStar;
  Eigen::MatrixXd mN;
  Eigen::MatrixXd mB;

  // Matrices to pass to solver
  Eigen::MatrixXd mA;
  Eigen::VectorXd mQBar;
  Eigen::VectorXd mX;

  std::vector<Eigen::VectorXd> mContactForces;
  // solved constraint force in generalized coordinates;
  // mTotalConstrForces[i] is the constraint force for the ith skeleton
  std::vector<Eigen::VectorXd> mTotalConstrForces;
  // constraints
  std::vector<Constraint*> mConstraints;
  int mTotalRows;

  Eigen::MatrixXd mZ;  // N x N, symmetric (only lower triangle filled)
  Eigen::VectorXd mTauHat;  // M x 1
  Eigen::MatrixXd mGInv;  // M x M, symmetric (only lower triangle filled)
  std::vector<Eigen::MatrixXd> mJMInv;  // M x N
  std::vector<Eigen::MatrixXd> mJ;  // M x N
  std::vector<Eigen::MatrixXd> mPreJ;  // M x N
  Eigen::VectorXd mC;  // M * 1
  Eigen::VectorXd mCDot;  // M * 1
  // if dof i hits upper limit, we store this information as
  // mLimitingDofIndex.push_back(i+1), if dof i hits lower limite,
  // mLimitingDofIndex.push_back(-(i+1));
  std::vector<int> mLimitingDofIndex;
  bool mUseODELCPSolver;

  // TODO: this map needs to be rebuilt when the order of skeletons changes
  std::map<Constraint*, Eigen::Vector2i> mSkeletonIDMap;

  Eigen::VectorXd mContactERVelocity; // Contact error reduction velocity
  double mContactErrorAllowance;      // Contact error allowance
  double mContactERP;                 // Contact error reduction parameter [0.0, 1.0]
  double mMaxContactERV;              // Maximum contact error reduction velocity

  std::vector<double> mJointLimitERVelocity;  // Joint limit error reduction velocity
  double mJointLimitErrorAllowance;           // Joint limit error allowance
  double mJointLimitERP;                      // Joint limit error reduction parameter [0.0, 1.0]
  double mMaxJointLimitERV;                   // Maximum joint limit error reduction velocity
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINTDYNAMICS_H_
