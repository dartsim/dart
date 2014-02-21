/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef SOFT_DYNAMICS_POINTMASS_H_
#define SOFT_DYNAMICS_POINTMASS_H_

#include <vector>
#include <Eigen/Dense>
#include <dart/dynamics/GenCoordSystem.h>

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class EllipsoidShape;
class SoftBodyNode;

/// \brief
class PointMass : public GenCoordSystem
{
public:
  friend class SoftBodyNode;

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief Default constructor
  explicit PointMass(SoftBodyNode* _softBodyNode);

  /// \brief Default destructor
  virtual ~PointMass();

  /// \brief
  void setMass(double _mass);

  /// \brief
  double getMass() const;

  /// \brief
  void addConnectedPointMass(PointMass* _pointMass);

  /// \brief
  int getNumConnectedPointMasses() const;

  /// \brief
  PointMass* getConnectedPointMass(int _idx) const;

  /// \brief Set whether this point mass is colliding with others.
  /// \param[in] True if this point mass is colliding.
  void setColliding(bool _isColliding);

  /// \brief Get whether this point mass is colliding with others.
  /// \return True if this point mass is colliding.
  bool isColliding();

  /// \brief Add linear Cartesian force to this node.
  /// \param[in] _force External force.
  /// \param[in] _isForceLocal True if _force's reference frame is of the parent
  ///                          soft body node. False if _force's reference frame
  ///                          is of the world.
  void addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal = false);

  /// \brief
  void clearExtForce();

  /// \brief
  void setRestingPosition(const Eigen::Vector3d& _p);

  /// \brief
  const Eigen::Vector3d& getRestingPosition() const;

  /// \brief
  const Eigen::Vector3d& getLocalPosition() const;

  /// \brief
  const Eigen::Vector3d& getWorldPosition() const;

  /// \todo Temporary function.
  Eigen::Matrix<double, 3, Eigen::Dynamic> getBodyJacobian();
  Eigen::Matrix<double, 3, Eigen::Dynamic> getWorldJacobian();

  SoftBodyNode* getParentSoftBodyNode() const;

  /// \brief The number of the generalized coordinates by which this node is
  ///        affected.
  int getNumDependentGenCoords() const;

  /// \brief Return a generalized coordinate index from the array index
  ///        (< getNumDependentDofs).
  int getDependentGenCoord(int _arrayIndex) const;

  /// \brief Get the generalized velocity at the position of this point mass
  ///        where the velocity is expressed in the parent soft body node frame.
  const Eigen::Vector3d& getBodyVelocity() const;

  /// \brief Get the generalized velocity at the position of this point mass
  ///        where the velocity is expressed in the world frame.
  Eigen::Vector3d getWorldVelocity() const;

  /// \brief Get the generalized acceleration at the position of this point mass
  ///        where the acceleration is expressed in the parent soft body node
  ///        frame.
  const Eigen::Vector3d& getBodyAcceleration() const;

  /// \brief Get the generalized acceleration at the position of this point mass
  ///        where the acceleration is expressed in the world frame.
  Eigen::Vector3d getWorldAcceleration() const;

protected:
  /// \brief
  void init();

  /// \brief
  void updateTransform();

  /// \brief
  void updateVelocity();

  /// \brief
  void updateEta();

  /// \brief
  void updateAcceleration();

  /// \brief
  void updateBodyForce(const Eigen::Vector3d& _gravity,
                       bool _withExternalForces = false);

  /// \brief
  void updateArticulatedInertia(double _dt);

  /// \brief
  void updateGeneralizedForce(bool _withDampingForces = false);

  /// \brief
  void updateBiasForce(double _dt, const Eigen::Vector3d& _gravity);

  /// \brief
  void update_ddq();

  /// \brief
  void update_F_fs();

  /// \brief
  void updateMassMatrix();

  /// \brief
  void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);

  /// \brief
  void aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                              double _timeStep);

  /// \brief
  void updateInvMassMatrix();

  /// \brief
  void updateInvAugMassMatrix();

  /// \brief
  void aggregateInvMassMatrix(Eigen::MatrixXd* _MInvCol, int _col);

  /// \brief
  void aggregateInvAugMassMatrix(Eigen::MatrixXd* _MInvCol, int _col,
                                 double _timeStep);

  /// \brief
  void aggregateGravityForceVector(Eigen::VectorXd* _g,
                                   const Eigen::Vector3d& _gravity);

  /// \brief
  void updateCombinedVector();

  /// \brief
  void aggregateCombinedVector(Eigen::VectorXd* _Cg,
                               const Eigen::Vector3d& _gravity);

  /// \brief Aggregate the external forces mFext in the generalized
  ///        coordinates recursively.
  void aggregateExternalForces(Eigen::VectorXd* _Fext);

  //-------------------- Cache Data for Mass Matrix ----------------------------
  /// \brief
  Eigen::Vector3d mM_dV;

  /// \brief
  Eigen::Vector3d mM_F;

  //----------------- Cache Data for Mass Inverse Matrix -----------------------
  /// \brief
  Eigen::Vector3d mInvM_beta;

  //---------------- Cache Data for Gravity Force Vector -----------------------
  /// \brief
  Eigen::Vector3d mG_F;

  //------------------- Cache Data for Combined Vector -------------------------
  /// \brief
  Eigen::Vector3d mCg_dV;

  /// \brief
  Eigen::Vector3d mCg_F;

  //---------------------------- Rendering -------------------------------------
  /// \brief
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true) const;

protected:
  /// \brief
  GenCoord mCoordinate[3];

  /// \brief Mass.
  double mMass;

  /// \brief Current position viewed in world frame.
  Eigen::Vector3d mW;

  /// \brief Current position viewed in parent soft body node frame.
  Eigen::Vector3d mX;

  /// \brief Resting postion viewed in parent soft body node frame.
  Eigen::Vector3d mX0;

  /// \brief Current velocity viewed in parent soft body node frame.
  Eigen::Vector3d mV;

  /// \brief
  Eigen::Vector3d mEta;

  /// \brief
  Eigen::Vector3d mAlpha;

  /// \brief
  Eigen::Vector3d mBeta;

  /// \brief Current acceleration viewed in parent body node frame.
  Eigen::Vector3d mdV;

  /// \brief
  Eigen::Vector3d mF;

  /// \brief
  double mPsi;

  /// \brief
  double mImplicitPsi;

  /// \brief
  double mPi;

  /// \brief
  double mImplicitPi;

  /// \brief Bias force
  Eigen::Vector3d mB;

  /// \brief
  SoftBodyNode* mParentSoftBodyNode;

  /// \brief
  std::vector<PointMass*> mConnectedPointMasses;

  /// \brief External force.
  Eigen::Vector3d mFext;

  /// \brief A increasingly sorted list of dependent dof indices.
  std::vector<int> mDependentGenCoordIndices;

  /// \brief Whether the node is currently in collision with another node.
  bool mIsColliding;

private:
  EllipsoidShape* mShape;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//struct PointMassPair
//{
//  PointMass* pm1;
//  PointMass* pm2;
//};

}  // namespace dynamics
}  // namespace dart

#endif  // SOFT_DYNAMICS_POINTMASS_H_
