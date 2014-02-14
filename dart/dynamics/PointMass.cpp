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

#include "dart/dynamics/PointMass.h"

#include <dart/common/Console.h>
#include <dart/math/Geometry.h>
#include <dart/math/Helpers.h>
#include <dart/dynamics/EllipsoidShape.h>
#include <dart/renderer/RenderInterface.h>

#include "dart/dynamics/SoftBodyNode.h"

namespace dart {
namespace dynamics {

PointMass::PointMass(SoftBodyNode* _softBodyNode)
  : GenCoordSystem(),
    mMass(0.0005),
    mW(Eigen::Vector3d::Zero()),
    mX(Eigen::Vector3d::Zero()),
    mX0(Eigen::Vector3d::Zero()),
    mV(Eigen::Vector3d::Zero()),
    mEta(Eigen::Vector3d::Zero()),
    mAlpha(Eigen::Vector3d::Zero()),
    mBeta(Eigen::Vector3d::Zero()),
    mdV(Eigen::Vector3d::Zero()),
    mF(Eigen::Vector3d::Zero()),
    mB(Eigen::Vector3d::Zero()),
    mParentSoftBodyNode(_softBodyNode),
    mFext(Eigen::Vector3d::Zero()),
    mShape(new EllipsoidShape(Eigen::Vector3d(0.01, 0.01, 0.01))),
    mIsColliding(false)
{
  assert(mParentSoftBodyNode != NULL);

  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);
}

PointMass::~PointMass()
{
  delete mShape;
}

void PointMass::setMass(double _mass)
{
  assert(0.0 < _mass);
  mMass = _mass;
}

double PointMass::getMass() const
{
  return mMass;
}

void PointMass::addConnectedPointMass(PointMass* _pointMass)
{
  assert(_pointMass != NULL);

  mConnectedPointMasses.push_back(_pointMass);
}

int PointMass::getNumConnectedPointMasses() const
{
  return mConnectedPointMasses.size();
}

PointMass*PointMass::getConnectedPointMass(int _idx) const
{
  assert(0 <= _idx && _idx < mConnectedPointMasses.size());

  return mConnectedPointMasses[_idx];
}

void PointMass::setColliding(bool _isColliding)
{
  mIsColliding = _isColliding;
}

bool PointMass::isColliding()
{
  return mIsColliding;
}

void PointMass::addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal)
{
  if (_isForceLocal)
  {
    mFext += _force;
  }
  else
  {
    mFext += mParentSoftBodyNode->getWorldTransform().linear().transpose()
             * _force;
  }
}

void PointMass::clearExtForce()
{
  mFext.setZero();
}

void PointMass::setRestingPosition(const Eigen::Vector3d& _p)
{
  mX0 = _p;
}

const Eigen::Vector3d& PointMass::getRestingPosition() const
{
  return mX0;
}

const Eigen::Vector3d& PointMass::getLocalPosition() const
{
  return mX;
}

const Eigen::Vector3d& PointMass::getWorldPosition() const
{
  return mW;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getBodyJacobian()
{
  assert(mParentSoftBodyNode != NULL);

  int dof = mParentSoftBodyNode->getNumDependentGenCoords();
  int totalDof = mParentSoftBodyNode->getNumDependentGenCoords() + 3;

  Eigen::Matrix<double, 3, Eigen::Dynamic> J
      = Eigen::MatrixXd::Zero(3, totalDof);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = mX;

  J.leftCols(dof)
      = math::AdInvTJac(
          T, mParentSoftBodyNode->getBodyJacobian()).bottomRows<3>();
  J.rightCols<3>() = Eigen::Matrix3d::Identity();

  return J;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> PointMass::getWorldJacobian()
{
  return mParentSoftBodyNode->getWorldTransform().linear()
      * getBodyJacobian();
}

SoftBodyNode* PointMass::getParentSoftBodyNode() const
{
  return mParentSoftBodyNode;
}

int PointMass::getNumDependentGenCoords() const
{
  return mDependentGenCoordIndices.size();
}

int PointMass::getDependentGenCoord(int _arrayIndex) const
{
  assert(0 <= _arrayIndex && _arrayIndex < mDependentGenCoordIndices.size());
  return mDependentGenCoordIndices[_arrayIndex];
}

const Eigen::Vector3d&PointMass::getBodyVelocity() const
{
  return mV;
}

Eigen::Vector3d PointMass::getWorldVelocity() const
{
  return mParentSoftBodyNode->getWorldTransform().linear() * mV;
}

const Eigen::Vector3d& PointMass::getBodyAcceleration() const
{
  return mdV;
}

Eigen::Vector3d PointMass::getWorldAcceleration() const
{
  return mParentSoftBodyNode->getWorldTransform().linear() * mdV;
}

void PointMass::init()
{
  // Dependen generalized coordinate setting
  int parentDof = mParentSoftBodyNode->getNumDependentGenCoords();

  mDependentGenCoordIndices.resize(parentDof + 3);
  for (int i = 0; i < parentDof; ++i)
  {
    mDependentGenCoordIndices[i]
            = mParentSoftBodyNode->getDependentGenCoordIndex(i);
  }

  mDependentGenCoordIndices[parentDof]     = mCoordinate[0].getSkeletonIndex();
  mDependentGenCoordIndices[parentDof + 1] = mCoordinate[1].getSkeletonIndex();
  mDependentGenCoordIndices[parentDof + 2] = mCoordinate[2].getSkeletonIndex();
}

void PointMass::updateTransform()
{
  // Local transpose
  mX = get_q() + mX0;
  assert(!math::isNan(mX));

  // World transpose
  mW = mParentSoftBodyNode->getWorldTransform().translation()
       + mParentSoftBodyNode->getWorldTransform().linear() * mX;
  assert(!math::isNan(mW));
}

void PointMass::updateVelocity()
{
  // v = w(parent) x mX + v(parent) + dq
  mV = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mX)
       + mParentSoftBodyNode->getBodyVelocity().tail<3>()
       + get_dq();
  assert(!math::isNan(mV));
}

void PointMass::updateEta()
{
  // eta = w(parent) x dq
  Eigen::Vector3d dq = get_dq();
  mEta = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(dq);
  assert(!math::isNan(mEta));
}

void PointMass::updateAcceleration()
{
  // dv = dw(parent) x mX + dv(parent) + eata + ddq
  mdV = mParentSoftBodyNode->getBodyAcceleration().head<3>().cross(mX) +
        mParentSoftBodyNode->getBodyAcceleration().tail<3>() +
        mEta + get_ddq();
  assert(!math::isNan(mdV));
}

void PointMass::updateBodyForce(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces)
{
  // f = m*dv + w(parent) x m*v - fext
  mF.noalias() = mMass * mdV;
  mF += mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass * mV)
        - mFext;
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mF -= mMass * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
                   * _gravity);
  }
  assert(!math::isNan(mF));
}

void PointMass::updateArticulatedInertia(double _dt)
{
  // Articulated inertia
  // - Do nothing

  // Cache data: PsiK and Psi
  mPsi = 1.0 / mMass;
  mImplicitPsi
      = 1.0 / (mMass
               + _dt * mParentSoftBodyNode->getDampingCoefficient()
               + _dt * _dt * mParentSoftBodyNode->getVertexSpringStiffness());
  assert(!std::isnan(mImplicitPsi));

  // Cache data: AI_S_Psi
  // - Do nothing

  // Cache data: Pi
  mPi         = mMass - mMass * mMass * mPsi;
  mImplicitPi = mMass - mMass * mMass * mImplicitPsi;
  assert(!std::isnan(mPi));
  assert(!std::isnan(mImplicitPi));
}

void PointMass::updateGeneralizedForce(bool _withDampingForces)
{
  // tau = f
  set_tau(mF);
}

void PointMass::updateBiasForce(double _dt, const Eigen::Vector3d& _gravity)
{
  // B = w(parent) x m*v - fext - fgravity
  // - w(parent) x m*v - fext
  mB = mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass*mV) - mFext;
  // - fgravity
  if (mParentSoftBodyNode->getGravityMode() == true)
  {
    mB -= mMass
          * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
             * _gravity);
  }
  assert(!math::isNan(mB));

  // Cache data: alpha
  double kv = mParentSoftBodyNode->getVertexSpringStiffness();
  double ke = mParentSoftBodyNode->getEdgeSpringStiffness();
  double kd = mParentSoftBodyNode->getDampingCoefficient();
  int nN = mConnectedPointMasses.size();
  mAlpha = get_tau()
           - (kv + nN * ke) * get_q()
           - (_dt * (kv + nN * ke) + kd) * get_dq()
           - mMass * mEta
           - mB;
  for (int i = 0; i < mConnectedPointMasses.size(); ++i)
  {
    mAlpha += ke * (mConnectedPointMasses[i]->get_q()
                    + _dt * mConnectedPointMasses[i]->get_dq());
  }
  assert(!math::isNan(mAlpha));

  // Cache data: beta
  mBeta = mB;
  mBeta.noalias() += mMass * (mEta + mImplicitPsi * mAlpha);
  assert(!math::isNan(mBeta));
}

void PointMass::update_ddq()
{
  // ddq = imp_psi*(alpha - m*(dw(parent) x mX + dv(parent))
  Eigen::Vector3d ddq =
      mImplicitPsi
      * (mAlpha - mMass
         * (mParentSoftBodyNode->getBodyAcceleration().head<3>().cross(mX)
            + mParentSoftBodyNode->getBodyAcceleration().tail<3>()));
  set_ddq(ddq);
  assert(!math::isNan(ddq));

  // Update dv
  updateAcceleration();
}

void PointMass::update_F_fs()
{
  // f = m*dv + B
  mF = mB;
  mF.noalias() += mMass * mdV;
  assert(!math::isNan(mF));
}

void PointMass::updateMassMatrix()
{
  mM_dV = get_ddq()
          + mParentSoftBodyNode->mM_dV.head<3>().cross(mX)
          + mParentSoftBodyNode->mM_dV.tail<3>();
  assert(!math::isNan(mM_dV));
}

void PointMass::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  mM_F.noalias() = mMass * mM_dV;
  _MCol->block<3, 1>(iStart, _col).noalias() = mM_F;
}

void PointMass::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                       double _timeStep)
{
  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  mM_F.noalias() = mMass * mM_dV;

  double d = mParentSoftBodyNode->getDampingCoefficient();
  double kv = mParentSoftBodyNode->getVertexSpringStiffness();
  _MCol->block<3, 1>(iStart, _col).noalias()
      = mM_F + (_timeStep * _timeStep * kv + _timeStep * d) * get_ddq();
}

void PointMass::updateInvMassMatrix()
{
  mInvM_beta = get_tau();
}

void PointMass::updateInvAugMassMatrix()
{
  mInvM_beta = mMass * mImplicitPsi * get_tau();
}

void PointMass::aggregateInvMassMatrix(Eigen::MatrixXd* _MInvCol, int _col)
{
  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  _MInvCol->block<3, 1>(iStart, _col)
      = mPsi * get_tau()
        - mParentSoftBodyNode->mInvM_U.head<3>().cross(mX)
        - mParentSoftBodyNode->mInvM_U.tail<3>();
}

void PointMass::aggregateInvAugMassMatrix(Eigen::MatrixXd* _MInvCol, int _col,
                                          double _timeStep)
{
  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  _MInvCol->block<3, 1>(iStart, _col)
      = mImplicitPsi
        * (get_tau()
           - mMass * (mParentSoftBodyNode->mInvM_U.head<3>().cross(mX)
                      + mParentSoftBodyNode->mInvM_U.tail<3>()));
}

void PointMass::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                            const Eigen::Vector3d& _gravity)
{
  mG_F = mMass * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
                  * _gravity);

  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  _g->segment<3>(iStart) = mG_F;
}

void PointMass::updateCombinedVector()
{
  mCg_dV = mEta
           + mParentSoftBodyNode->mCg_dV.head<3>().cross(mX)
           + mParentSoftBodyNode->mCg_dV.tail<3>();
}

void PointMass::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                        const Eigen::Vector3d& _gravity)
{
  mCg_F.noalias() = mMass * mCg_dV;
  mCg_F -= mMass
           * (mParentSoftBodyNode->getWorldTransform().linear().transpose()
              * _gravity);
  mCg_F += mParentSoftBodyNode->getBodyVelocity().head<3>().cross(mMass * mV);

  // Assign
  // We assume that the three generalized coordinates are in a row.
  int iStart = getGenCoord(0)->getSkeletonIndex();
  _Cg->segment<3>(iStart) = mCg_F;
}

void PointMass::aggregateExternalForces(Eigen::VectorXd* _Fext)
{
  int iStart = getGenCoord(0)->getSkeletonIndex();
  _Fext->segment<3>(iStart) = mFext;
}

void PointMass::draw(renderer::RenderInterface* _ri,
                     const Eigen::Vector4d& _color,
                     bool _useDefaultColor) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
//  mParentJoint->applyGLTransform(_ri);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = mX;
  _ri->transform(T);
  Eigen::Vector4d color1;
  color1 << 0.8, 0.3, 0.3, 1.0;
  mShape->draw(_ri, color1, false);
  _ri->popMatrix();

//  _ri->pushName((unsigned)mID);
  _ri->pushMatrix();
  T.translation() = mX0;
  _ri->transform(T);
  Eigen::Vector4d color2;
  color2 << 0.3, 0.8, 0.3, 1.0;
  mShape->draw(_ri, color2, false);
  _ri->popMatrix();
//  _ri->popName();

}

}  // namespace dynamics
}  // namespace dart
