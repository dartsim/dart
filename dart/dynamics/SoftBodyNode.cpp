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

#include "dart/dynamics/SoftBodyNode.h"

#include <string>
#include <vector>

#include "dart/math/Helpers.h"
#include "dart/common/Console.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/renderer/LoadOpengl.h"
#include "dart/renderer/RenderInterface.h"

#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftMeshShape.h"

namespace dart {
namespace dynamics {

//==============================================================================
SoftBodyNode::SoftBodyNode(const std::string& _name)
  : BodyNode(_name),
    mKv(DART_DEFAULT_VERTEX_STIFFNESS),
    mKe(DART_DEFAULT_EDGE_STIFNESS),
    mDampCoeff(DART_DEFAULT_DAMPING_COEFF),
    mSoftVisualShape(NULL),
    mSoftCollShape(NULL)
{
}

//==============================================================================
SoftBodyNode::~SoftBodyNode()
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    delete mPointMasses[i];
}

//==============================================================================
size_t SoftBodyNode::getNumPointMasses() const
{
  return mPointMasses.size();
}

//==============================================================================
PointMass* SoftBodyNode::getPointMass(size_t _idx) const
{
  assert(0 <= _idx && _idx < mPointMasses.size());
  return mPointMasses[_idx];
}

//==============================================================================
void SoftBodyNode::init(Skeleton* _skeleton)
{
  BodyNode::init(_skeleton);

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses[i]->init();

//  //----------------------------------------------------------------------------
//  // Visualization shape
//  //----------------------------------------------------------------------------
//  assert(mSoftVisualShape == NULL);
//  mSoftVisualShape = new SoftMeshShape(this);
//  BodyNode::addVisualizationShape(mSoftVisualShape);

//  //----------------------------------------------------------------------------
//  // Collision shape
//  //----------------------------------------------------------------------------
//  assert(mSoftCollShape == NULL);
//  mSoftCollShape = new SoftMeshShape(this);
//  BodyNode::addCollisionShape(mSoftCollShape);
}

//==============================================================================
//void SoftBodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords)
//{
//  BodyNode::aggregateGenCoords(_genCoords);
//  aggregatePointMassGenCoords(_genCoords);
//}

//==============================================================================
//void SoftBodyNode::aggregatePointMassGenCoords(
//    std::vector<GenCoord*>* _genCoords)
//{
//  for (size_t i = 0; i < getNumPointMasses(); ++i)
//  {
//    PointMass* pointMass = getPointMass(i);
//    for (int j = 0; j < pointMass->getNumDofs(); ++j)
//    {
//      GenCoord* genCoord = pointMass->getGenCoord(j);
//      genCoord->setSkeletonIndex(_genCoords->size());
//      _genCoords->push_back(genCoord);
//    }
//  }
//}

//==============================================================================
double SoftBodyNode::getMass() const
{
  double totalMass = BodyNode::getMass();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    totalMass += mPointMasses.at(i)->getMass();

  return totalMass;
}

//==============================================================================
void SoftBodyNode::setVertexSpringStiffness(double _kv)
{
  assert(0.0 <= _kv);
  mKv = _kv;
}

//==============================================================================
double SoftBodyNode::getVertexSpringStiffness() const
{
  return mKv;
}

//==============================================================================
void SoftBodyNode::setEdgeSpringStiffness(double _ke)
{
  assert(0.0 <= _ke);
  mKe = _ke;
}

//==============================================================================
double SoftBodyNode::getEdgeSpringStiffness() const
{
  return mKe;
}

//==============================================================================
void SoftBodyNode::setDampingCoefficient(double _damp)
{
  assert(_damp >= 0.0);
  mDampCoeff = _damp;
}

//==============================================================================
double SoftBodyNode::getDampingCoefficient() const
{
  return mDampCoeff;
}

//==============================================================================
void SoftBodyNode::removeAllPointMasses()
{
  mPointMasses.clear();
}

//==============================================================================
void SoftBodyNode::addPointMass(PointMass* _pointMass)
{
  assert(_pointMass != NULL);
  mPointMasses.push_back(_pointMass);
}

//==============================================================================
void SoftBodyNode::connectPointMasses(size_t _idx1, size_t _idx2)
{
  assert(_idx1 != _idx2);
  assert(0 <= _idx1 && _idx1 < mPointMasses.size());
  assert(0 <= _idx2 && _idx2 < mPointMasses.size());
  mPointMasses[_idx1]->addConnectedPointMass(mPointMasses[_idx2]);
  mPointMasses[_idx2]->addConnectedPointMass(mPointMasses[_idx1]);
}

//==============================================================================
void SoftBodyNode::addFace(const Eigen::Vector3i& _face)
{
  assert(_face[0] != _face[1]);
  assert(_face[1] != _face[2]);
  assert(_face[2] != _face[0]);
  assert(0 <= _face[0] && _face[0] < math::castUIntToInt(mPointMasses.size()));
  assert(0 <= _face[1] && _face[1] < math::castUIntToInt(mPointMasses.size()));
  assert(0 <= _face[2] && _face[2] < math::castUIntToInt(mPointMasses.size()));
  mFaces.push_back(_face);
}

//==============================================================================
const Eigen::Vector3i& SoftBodyNode::getFace(size_t _idx) const
{
  assert(0 <= _idx && _idx < mFaces.size());
  return mFaces[_idx];
}

//==============================================================================
size_t SoftBodyNode::getNumFaces()
{
  return mFaces.size();
}

//==============================================================================
void SoftBodyNode::clearConstraintImpulse()
{
  BodyNode::clearConstraintImpulse();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearConstraintImpulse();
}

//==============================================================================
void SoftBodyNode::updateTransform()
{
  BodyNode::updateTransform();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateTransform();
}

//==============================================================================
void SoftBodyNode::updateVelocity()
{
  BodyNode::updateVelocity();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateVelocity();
}

//==============================================================================
void SoftBodyNode::updatePartialAcceleration()
{
  BodyNode::updatePartialAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updatePartialAcceleration();
}

//==============================================================================
void SoftBodyNode::updateAcceleration()
{
  BodyNode::updateAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateAcceleration();
}

//==============================================================================
void SoftBodyNode::updateBodyWrench(const Eigen::Vector3d& _gravity,
                                   bool _withExternalForces)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBodyForce(_gravity, _withExternalForces);

  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  // Inertial force
  mF.noalias() = mI * mA;

  // External force
  if (_withExternalForces)
    mF -= mFext;

  // Verification
  assert(!math::isNan(mF));

  // Gravity force
  mF -= mFgravity;

  // Coriolis force
  mF -= math::dad(mV, mI * mV);

  //
  for (std::vector<BodyNode*>::iterator iChildBody = mChildBodyNodes.begin();
       iChildBody != mChildBodyNodes.end(); ++iChildBody)
  {
    Joint* childJoint = (*iChildBody)->getParentJoint();
    assert(childJoint != NULL);

    mF += math::dAdInvT(childJoint->getLocalTransform(),
                        (*iChildBody)->getBodyForce());
  }
//  for (size_t i = 0; i < mPointMasses.size(); i++)
//  {
//    mF.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mF);
//    mF.tail<3>() += mPointMasses[i]->mF;
//  }

  // TODO(JS): mWrench and mF are duplicated. Remove one of them.
  mParentJoint->mWrench = mF;

  // Verification
  assert(!math::isNan(mF));
}

//==============================================================================
void SoftBodyNode::updateGeneralizedForce(bool _withDampingForces)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateGeneralizedForce(_withDampingForces);

  BodyNode::updateGeneralizedForce(_withDampingForces);
}

//==============================================================================
void SoftBodyNode::updateArtInertia(double _timeStep)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateArticulatedInertia(_timeStep);

  assert(mParentJoint != NULL);

  // Set spatial inertia to the articulated body inertia
  mArtInertia = mI;
  mArtInertiaImplicit = mI;

  // and add child articulated body inertia
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildArtInertiaTo(
          mArtInertia, (*it)->mArtInertia);
    (*it)->getParentJoint()->addChildArtInertiaImplicitTo(
          mArtInertiaImplicit, (*it)->mArtInertiaImplicit);
  }

  //
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    _addPiToArtInertia(mPointMasses[i]->mX, mPointMasses[i]->mPi);
    _addPiToArtInertiaImplicit(mPointMasses[i]->mX, mPointMasses[i]->mImplicitPi);
  }

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));

  // Update parent joint's inverse of projected articulated body inertia
  mParentJoint->updateInvProjArtInertia(mArtInertia);
  mParentJoint->updateInvProjArtInertiaImplicit(mArtInertiaImplicit, _timeStep);

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));
}

//==============================================================================
void SoftBodyNode::updateBiasForce(const Eigen::Vector3d& _gravity,
                                   double _timeStep)
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBiasForce(_timeStep, _gravity);

  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  // Set bias force
  mBiasForce = -math::dad(mV, mI * mV) - mFext - mFgravity;

  // Verifycation
  assert(!math::isNan(mBiasForce));

  // And add child bias force
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceTo(mBiasForce,
                                                 (*it)->mArtInertiaImplicit,
                                                 (*it)->mBiasForce,
                                                 (*it)->mPartialAcceleration);
  }

  //
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    mBiasForce.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mBeta);
    mBiasForce.tail<3>() += mPointMasses[i]->mBeta;
  }

  // Verifycation
  assert(!math::isNan(mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  mParentJoint->updateTotalForce(
        mArtInertiaImplicit * mPartialAcceleration + mBiasForce, _timeStep);
}

//==============================================================================
void SoftBodyNode::updateJointAndBodyAcceleration()
{
  BodyNode::updateJointAndBodyAcceleration();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateJointAndBodyAcceleration();
}

//==============================================================================
void SoftBodyNode::updateTransmittedWrench()
{
  BodyNode::updateTransmittedWrench();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateTransmittedForce();
}

//==============================================================================
void SoftBodyNode::updateBiasImpulse()
{
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBiasImpulse();

  // Update impulsive bias force
  mBiasImpulse = -mConstraintImpulse;
//  assert(mImpFext == Eigen::Vector6d::Zero());

  // And add child bias impulse
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasImpulseTo(mBiasImpulse,
                                                   (*it)->mArtInertia,
                                                   (*it)->mBiasImpulse);
  }

  // TODO(JS):
  for (size_t i = 0; i < mPointMasses.size(); i++)
  {
    mBiasImpulse.head<3>() += mPointMasses[i]->mX.cross(mPointMasses[i]->mImpBeta);
    mBiasImpulse.tail<3>() += mPointMasses[i]->mImpBeta;
  }

  // Verification
  assert(!math::isNan(mBiasImpulse));

  // Update parent joint's total force
  mParentJoint->updateTotalImpulse(mBiasImpulse);
}

//==============================================================================
void SoftBodyNode::updateJointVelocityChange()
{
  BodyNode::updateJointVelocityChange();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateJointVelocityChange();
}

//==============================================================================
//void SoftBodyNode::updateBodyVelocityChange()
//{
//  BodyNode::updateBodyVelocityChange();

//  for (size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateBodyVelocityChange();
//}

//==============================================================================
void SoftBodyNode::updateBodyImpForceFwdDyn()
{
  BodyNode::updateBodyImpForceFwdDyn();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateBodyImpForceFwdDyn();
}

//==============================================================================
void SoftBodyNode::updateConstrainedJointAndBodyAcceleration(double _timeStep)
{
  BodyNode::updateConstrainedJointAndBodyAcceleration(_timeStep);

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateConstrainedJointAndBodyAcceleration(_timeStep);
}

//==============================================================================
void SoftBodyNode::updateConstrainedTransmittedForce(double _timeStep)
{
  BodyNode::updateConstrainedTransmittedForce(_timeStep);
}

//==============================================================================
void SoftBodyNode::updateMassMatrix()
{
  BodyNode::updateMassMatrix();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateMassMatrix();
}

//==============================================================================
void SoftBodyNode::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateMassMatrix(_MCol, _col);

  //----------------------- SoftBodyNode Part ----------------------------------
  //
  mM_F.noalias() = mI * mM_dV;

  // Verification
  assert(!math::isNan(mM_F));

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }

  //
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mM_F.head<3>() += (*it)->mX.cross((*it)->mM_F);
    mM_F.tail<3>() += (*it)->mM_F;
  }

  // Verification
  assert(!math::isNan(mM_F));

  //
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _MCol->block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getLocalJacobian().transpose() * mM_F;
  }
}

//==============================================================================
void SoftBodyNode::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                          double _timeStep)
{
  // TODO(JS): Need to be reimplemented

  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateAugMassMatrix(_MCol, _col, _timeStep);

  //----------------------- SoftBodyNode Part ----------------------------------
  mM_F.noalias() = mI * mM_dV;
  assert(!math::isNan(mM_F));

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mM_F.head<3>() += (*it)->mX.cross((*it)->mM_F);
    mM_F.tail<3>() += (*it)->mM_F;
  }
  assert(!math::isNan(mM_F));

  size_t dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (size_t i = 0; i < dof; ++i)
    {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }
    int iStart = mParentJoint->getIndexInSkeleton(0);

    // TODO(JS): Not recommended to use Joint::getAccelerations
    _MCol->block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getLocalJacobian().transpose() * mM_F
          + D * (_timeStep * mParentJoint->getAccelerations())
          + K * (_timeStep * _timeStep * mParentJoint->getAccelerations());
  }
}

//==============================================================================
void SoftBodyNode::updateInvMassMatrix()
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateInvMassMatrix();

  //----------------------- SoftBodyNode Part ----------------------------------
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceForInvMassMatrix(
          mInvM_c, (*it)->mArtInertia, (*it)->mInvM_c);
  }

  //
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mInvM_c.head<3>() += (*it)->mX.cross((*it)->mBiasForceForInvMeta);
    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
  }

  // Verification
  assert(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::updateInvAugMassMatrix()
{
  BodyNode::updateInvAugMassMatrix();

//  //------------------------ PointMass Part ------------------------------------
////  for (size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->updateInvAugMassMatrix();

//  //----------------------- SoftBodyNode Part ----------------------------------
//  //
//  mInvM_c.setZero();

//  //
//  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    (*it)->getParentJoint()->addChildBiasForceForInvAugMassMatrix(
//          mInvM_c, (*it)->mArtInertiaImplicit, (*it)->mInvM_c);
//  }

//  //
////  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
////       it != mPointMasses.end(); ++it)
////  {
////    mInvM_c.head<3>() += (*it)->mX.cross((*it)->mBiasForceForInvMeta);
////    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
////  }

//  // Verification
//  assert(!math::isNan(mInvM_c));

//  // Update parent joint's total force for inverse mass matrix
//  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, mArtInertia, mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, mArtInertia, Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

  //
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateInvMassMatrix(_InvMCol, _col);
}

//==============================================================================
void SoftBodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol,
                                             int _col,
                                             double _timeStep)
{
  BodyNode::aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);

//  if (mParentBodyNode)
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, mArtInertiaImplicit, mParentBodyNode->mInvM_U);

//    //
//    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
//  }
//  else
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, mArtInertiaImplicit, Eigen::Vector6d::Zero());

//    //
//    mInvM_U.setZero();
//  }

//  //
//  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

//  //
////  for (size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);
}

//==============================================================================
void SoftBodyNode::aggregateCoriolisForceVector(Eigen::VectorXd* _C)
{
  BodyNode::aggregateCoriolisForceVector(_C);
}

//==============================================================================
void SoftBodyNode::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                               const Eigen::Vector3d& _gravity)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateGravityForceVector(_g, _gravity);

  //----------------------- SoftBodyNode Part ----------------------------------
  if (mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(mW, _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mG_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                          (*it)->mG_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mG_F.head<3>() += (*it)->mX.cross((*it)->mG_F);
    mG_F.tail<3>() += (*it)->mG_F;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd g = -(mParentJoint->getLocalJacobian().transpose() * mG_F);
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _g->segment(iStart, nGenCoords) = g;
  }
}

//==============================================================================
void SoftBodyNode::updateCombinedVector()
{
  BodyNode::updateCombinedVector();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateCombinedVector();
}

//==============================================================================
void SoftBodyNode::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                           const Eigen::Vector3d& _gravity)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateCombinedVector(_Cg, _gravity);

  //----------------------- SoftBodyNode Part ----------------------------------
  // H(i) = I(i) * W(i) -
  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
  if (mGravityMode == true)
    mFgravity = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  mCg_F = mI * mCg_dV;
  mCg_F -= mFgravity;
  mCg_F -= math::dad(mV, mI * mV);

  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mCg_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                           (*it)->mCg_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mCg_F.head<3>() += (*it)->mX.cross((*it)->mCg_F);
    mCg_F.tail<3>() += (*it)->mCg_F;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Cg = mParentJoint->getLocalJacobian().transpose() * mCg_F;
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _Cg->segment(iStart, nGenCoords) = Cg;
  }
}

//==============================================================================
void SoftBodyNode::aggregateExternalForces(Eigen::VectorXd* _Fext)
{
  //------------------------ PointMass Part ------------------------------------
  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateExternalForces(_Fext);

  //----------------------- SoftBodyNode Part ----------------------------------
  mFext_F = mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                             (*it)->mFext_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mFext_F.head<3>() += (*it)->mX.cross((*it)->mFext);
    mFext_F.tail<3>() += (*it)->mFext;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Fext
        = mParentJoint->getLocalJacobian().transpose() * mFext_F;
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _Fext->segment(iStart, nGenCoords) = Fext;
  }
}

//==============================================================================
void SoftBodyNode::clearExternalForces()
{
  BodyNode::clearExternalForces();

  for (size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearExtForce();
}

//==============================================================================
void SoftBodyNode::draw(renderer::RenderInterface* _ri,
                        const Eigen::Vector4d& _color,
                        bool _useDefaultColor,
                        int _depth) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
  mParentJoint->applyGLTransform(_ri);

  _ri->pushName((unsigned)mID);
  // rigid body
  for (size_t i = 0; i < mVizShapes.size(); i++)
  {
    _ri->pushMatrix();
    mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }

  // vertex
//  if (_showPointMasses)
  {
    for (size_t i = 0; i < mPointMasses.size(); ++i)
    {
      _ri->pushMatrix();
      mPointMasses[i]->draw(_ri, _color, _useDefaultColor);
      _ri->popMatrix();
    }
  }

  // edges (mesh)
  Eigen::Vector4d fleshColor = _color;
  fleshColor[3] = 0.5;
  _ri->setPenColor(fleshColor);
//  if (_showMeshs)
  {
    Eigen::Vector3d pos;
    Eigen::Vector3d pos_normalized;
    for (size_t i = 0; i < mFaces.size(); ++i)
    {
      glEnable(GL_AUTO_NORMAL);
      glBegin(GL_TRIANGLES);

      pos = mPointMasses[mFaces[i](0)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](1)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](2)]->mX;
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      glEnd();
    }
  }

  _ri->popName();

  // render the subtree
  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
  {
    getChildBodyNode(i)->draw(_ri, _color, _useDefaultColor);
  }

  _ri->popMatrix();
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertia(const Eigen::Vector3d& _p, double _Pi)
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertia.topLeftCorner<3, 3>()    -= _Pi * tmp * tmp;
  mArtInertia.topRightCorner<3, 3>()   += _Pi * tmp;
  mArtInertia.bottomLeftCorner<3, 3>() -= _Pi * tmp;

  mArtInertia(3, 3) += _Pi;
  mArtInertia(4, 4) += _Pi;
  mArtInertia(5, 5) += _Pi;
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertiaImplicit(const Eigen::Vector3d& _p,
                                              double _ImplicitPi)
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertiaImplicit.topLeftCorner<3, 3>()    -= _ImplicitPi * tmp * tmp;
  mArtInertiaImplicit.topRightCorner<3, 3>()   += _ImplicitPi * tmp;
  mArtInertiaImplicit.bottomLeftCorner<3, 3>() -= _ImplicitPi * tmp;

  mArtInertiaImplicit(3, 3) += _ImplicitPi;
  mArtInertiaImplicit(4, 4) += _ImplicitPi;
  mArtInertiaImplicit(5, 5) += _ImplicitPi;
}

//==============================================================================
void SoftBodyNode::updateInertiaWithPointMass()
{
  // TODO(JS): Not implemented

  mI2 = mI;

  for (size_t i = 0; i < mPointMasses.size(); ++i)
  {

  }
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransfom,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nPointMasses = 8;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  restingPos[1] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  restingPos[2] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  restingPos[3] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  restingPos[4] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  restingPos[5] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  restingPos[6] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;
  restingPos[7] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;

  // Point masses
  dynamics::PointMass* newPointMass = NULL;
  for (size_t i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new PointMass(_softBodyNode);
    newPointMass->setRestingPosition(_localTransfom * restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // -- Bottoms
  _softBodyNode->connectPointMasses(0, 1);
  _softBodyNode->connectPointMasses(1, 3);
  _softBodyNode->connectPointMasses(3, 2);
  _softBodyNode->connectPointMasses(2, 0);

  // -- Tops
  _softBodyNode->connectPointMasses(4, 5);
  _softBodyNode->connectPointMasses(5, 7);
  _softBodyNode->connectPointMasses(7, 6);
  _softBodyNode->connectPointMasses(6, 4);

  // -- Sides
  _softBodyNode->connectPointMasses(0, 4);
  _softBodyNode->connectPointMasses(1, 5);
  _softBodyNode->connectPointMasses(2, 6);
  _softBodyNode->connectPointMasses(3, 7);

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  // -- +Z
  _softBodyNode->addFace(Eigen::Vector3i(1, 0, 2));  // 0
  _softBodyNode->addFace(Eigen::Vector3i(1, 2, 3));  // 1

  // -- -Z
  _softBodyNode->addFace(Eigen::Vector3i(5, 6, 4));  // 2
  _softBodyNode->addFace(Eigen::Vector3i(5, 7, 6));  // 3

  // -- -Y
  _softBodyNode->addFace(Eigen::Vector3i(0, 5, 4));  // 4
  _softBodyNode->addFace(Eigen::Vector3i(0, 1, 5));  // 5

  // -- +Y
  _softBodyNode->addFace(Eigen::Vector3i(1, 3, 7));  // 6
  _softBodyNode->addFace(Eigen::Vector3i(1, 7, 5));  // 7

  // -- -X
  _softBodyNode->addFace(Eigen::Vector3i(3, 2, 6));  // 8
  _softBodyNode->addFace(Eigen::Vector3i(3, 6, 7));  // 9

  // -- +X
  _softBodyNode->addFace(Eigen::Vector3i(2, 0, 4));  // 10
  _softBodyNode->addFace(Eigen::Vector3i(2, 4, 6));  // 11
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransfom,
                                const Eigen::Vector3i&   _frags,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  // Half size
  Eigen::Vector3d halfSize = 0.5 * _size;

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  assert(_frags[0] > 1 && _frags[1] > 1 && _frags[2] > 1);
  int nVertices = 2 * (_frags[0] * _frags[1])
                     + 2 * (_frags[1] * _frags[2])
                     + 2 * (_frags[2] * _frags[0]);
  Eigen::Vector3d interval(_size[0]/(_frags[0] - 1),
                           _size[1]/(_frags[1] - 1),
                           _size[2]/(_frags[2] - 1));

  // Mass per vertices
  double mass = _totalMass / nVertices;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nVertices,
                                          Eigen::Vector3d::Zero());

  int vIdx = 0;

  // +X side
  for (int k = 0; k < _frags[2]; ++k)    // z
  {
    for (int j = 0; j < _frags[1]; ++j)  // y
    {
      restingPos[vIdx++] <<  halfSize[0],
                            -halfSize[1] + j * interval[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // -X side
  for (int k = 0; k < _frags[2]; ++k)    // z
  {
    for (int j = 0; j < _frags[1]; ++j)  // y
    {
      restingPos[vIdx++] << -halfSize[0],
                            -halfSize[1] + j * interval[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Y side
  for (int i = 0; i < _frags[0]; ++i)    // x
  {
    for (int k = 0; k < _frags[2]; ++k)  // z
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                             halfSize[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Y side
  for (int i = 0; i < _frags[0]; ++i)    // x
  {
    for (int k = 0; k < _frags[2]; ++k)  // z
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Z side
  for (int j = 0; j < _frags[2]; ++j)    // y
  {
    for (int i = 0; i < _frags[1]; ++i)  // x
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1] + j * interval[1],
                             halfSize[2];
    }
  }

  // -Z side
  for (int j = 0; j < _frags[2]; ++j)    // y
  {
    for (int i = 0; i < _frags[1]; ++i)  // x
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1] + j * interval[1],
                             halfSize[2];
    }
  }

  // Point masses
  dynamics::PointMass* newPointMass = NULL;
  for (int i = 0; i < nVertices; ++i)
  {
    newPointMass = new PointMass(_softBodyNode);
    newPointMass->setRestingPosition(_localTransfom * restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int nFaces = 4 * ((_frags[0] - 1) * (_frags[1] - 1))
               + 4 * ((_frags[1] - 1) * (_frags[2] - 1))
               + 4 * ((_frags[2] - 1) * (_frags[0] - 1));
  std::vector<Eigen::Vector3i> faces(nFaces, Eigen::Vector3i::Zero());

  int fIdx = 0;
  int baseIdx = 0;
  Eigen::Vector3i fItr;

  // +X side faces
  for (int k = 0; k < _frags[2] - 1; ++k)    // z
  {
    for (int j = 0; j < _frags[1] - 1; ++j)  // y
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[1] * j + k;
      faces[fIdx][1] = baseIdx + _frags[1] * j + k + 1;
      faces[fIdx][2] = baseIdx + _frags[1] * (j + 1) + k;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[1] * (j + 1) + k + 1;
      faces[fIdx][1] = baseIdx + _frags[1] * (j + 1) + k ;
      faces[fIdx][2] = baseIdx + _frags[1] * j + k + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[1] * _frags[2];

  // -X side faces
  for (int k = 0; k < _frags[2] - 1; ++k)    // z
  {
    for (int j = 0; j < _frags[1] - 1; ++j)  // y
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[1] * j + k;
      faces[fIdx][1] = baseIdx + _frags[1] * (j + 1) + k;
      faces[fIdx][2] = baseIdx + _frags[1] * j + k + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[1] * (j + 1) + k + 1;
      faces[fIdx][1] = baseIdx + _frags[1] * j + k + 1;
      faces[fIdx][2] = baseIdx + _frags[1] * (j + 1) + k ;
      fIdx++;
    }
  }
  baseIdx += _frags[1] * _frags[2];

  // +Y side faces
  for (int i = 0; i < _frags[0] - 1; ++i)    // x
  {
    for (int k = 0; k < _frags[2] - 1; ++k)  // z
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[2] * k + i;
      faces[fIdx][1] = baseIdx + _frags[2] * k + i + 1;
      faces[fIdx][2] = baseIdx + _frags[2] * (k + 1) + i;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[2] * (k + 1) + i + 1;
      faces[fIdx][1] = baseIdx + _frags[2] * (k + 1) + i ;
      faces[fIdx][2] = baseIdx + _frags[2] * k + i + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[2] * _frags[0];

  // -Y side faces
  for (int i = 0; i < _frags[0] - 1; ++i)    // x
  {
    for (int k = 0; k < _frags[2] - 1; ++k)  // z
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[2] * k + i;
      faces[fIdx][1] = baseIdx + _frags[2] * (k + 1) + i;
      faces[fIdx][2] = baseIdx + _frags[2] * k + i + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[2] * (k + 1) + i + 1;
      faces[fIdx][1] = baseIdx + _frags[2] * k + i + 1;
      faces[fIdx][2] = baseIdx + _frags[2] * (k + 1) + i ;
      fIdx++;
    }
  }
  baseIdx += _frags[2] * _frags[0];

  // +Z side faces
  for (int j = 0; j < _frags[1] - 1; ++j)    // y
  {
    for (int i = 0; i < _frags[0] - 1; ++i)  // x
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[0] * i + j;
      faces[fIdx][1] = baseIdx + _frags[0] * i + j + 1;
      faces[fIdx][2] = baseIdx + _frags[0] * (i + 1) + j;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[0] * (i + 1) + j + 1;
      faces[fIdx][1] = baseIdx + _frags[0] * (i + 1) + j ;
      faces[fIdx][2] = baseIdx + _frags[0] * i + j + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[0] * _frags[1];

  // -Z side faces
  for (int j = 0; j < _frags[1] - 1; ++j)    // y
  {
    for (int i = 0; i < _frags[0] - 1; ++i)  // x
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[0] * i + j;
      faces[fIdx][1] = baseIdx + _frags[0] * (i + 1) + j;
      faces[fIdx][2] = baseIdx + _frags[0] * i + j + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[0] * (i + 1) + j + 1;
      faces[fIdx][1] = baseIdx + _frags[0] * i + j + 1;
      faces[fIdx][2] = baseIdx + _frags[0] * (i + 1) + j ;
      fIdx++;
    }
  }

  // Add to the soft body node
  for (int i = 0; i < nFaces; ++i)
  {
    _softBodyNode->addFace(faces[i]);
  }
}

//==============================================================================
void SoftBodyNodeHelper::setSinglePointMass(SoftBodyNode* _softBodyNode,
                                        double _totalMass,
                                        double _vertexStiffness,
                                        double _edgeStiffness,
                                        double _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nPointMasses = 1;\

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = Eigen::Vector3d(+0.1, +0.1, +0.1);

  // Point masses
  dynamics::PointMass* newPointMass = NULL;
  for (size_t i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new PointMass(_softBodyNode);
    newPointMass->setRestingPosition(restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }
}

//==============================================================================
void SoftBodyNodeHelper::setEllipsoid(SoftBodyNode*          _softBodyNode,
                                      const Eigen::Vector3d& _size,
                                      size_t                 _nSlices,
                                      size_t                 _nStacks,
                                      double                 _totalMass,
                                      double                 _vertexStiffness,
                                      double                 _edgeStiffness,
                                      double                 _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = (_nStacks - 1) * _nSlices + 2;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Point mass pointer
  PointMass* newPointMass = NULL;

  // Resting positions for each point mass
  // -- top
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, 0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);
  // middle
  float drho = (DART_PI / _nStacks);
  float dtheta = (DART_2PI / _nSlices);
  for (size_t i = 1; i < _nStacks; i++)
  {
    float rho = i * drho;
    float srho = (sin(rho));
    float crho = (cos(rho));

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = 0.5 * srho * stheta;
      float y = 0.5 * srho * ctheta;
      float z = 0.5 * crho;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * _size(0), y * _size(1), z * _size(2)));
      _softBodyNode->addPointMass(newPointMass);
    }
  }
  // bottom
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, -0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);


  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // a) longitudinal
  // -- top
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(0, i + 1);
  // -- middle
  for (size_t i = 0; i < _nStacks - 2; i++)
    for (size_t j = 0; j < _nSlices; j++)
      _softBodyNode->connectPointMasses(i*_nSlices + j + 1,
                                        (i + 1)*_nSlices + j + 1);
  // -- bottom
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses((_nStacks-1)*_nSlices + 1,
                                      (_nStacks-2)*_nSlices + i + 1);

  // b) latitudinal
  for (size_t i = 0; i < _nStacks - 1; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(i*_nSlices + j + 1, i*_nSlices + j + 2);
    }
    _softBodyNode->connectPointMasses((i+1)*_nSlices, i*_nSlices + 1);
  }

  // c) cross (shear)
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(i * _nSlices + j + 1,
                                        (i + 1) * _nSlices + j + 2);
      _softBodyNode->connectPointMasses(i * _nSlices + j + 2,
                                        (i + 1) * _nSlices + j + 1);
    }
    _softBodyNode->connectPointMasses((i+1)*_nSlices, (i+1)*_nSlices + 1);
    _softBodyNode->connectPointMasses(i*_nSlices + 1, (i+2)*_nSlices);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  meshIdx1 = 0;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  // middle
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = i*_nSlices + j + 1;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = i*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

      meshIdx1 = i*_nSlices + j + 2;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = (i + 1)*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
    }

    meshIdx1 = (i + 1)*_nSlices;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = i*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

    meshIdx1 = i*_nSlices + 1;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = (i + 2)*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }

  // bottom
  meshIdx1 = (_nStacks-1)*_nSlices + 1;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = (_nStacks-2)*_nSlices + i + 2;
    meshIdx3 = (_nStacks-2)*_nSlices + i + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = (_nStacks-2)*_nSlices + 2;
  meshIdx3 = (_nStacks-1)*_nSlices;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
}

//==============================================================================
void SoftBodyNodeHelper::setCylinder(SoftBodyNode* _softBodyNode,
                                     double _radius,
                                     double _height,
                                     size_t _nSlices,
                                     size_t _nStacks,
                                     size_t _nRings,
                                     double _totalMass,
                                     double _vertexStiffness,
                                     double _edgeStiffness,
                                     double _dampingCoeff)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Misc
  //----------------------------------------------------------------------------
  _softBodyNode->setVertexSpringStiffness(_vertexStiffness);
  _softBodyNode->setEdgeSpringStiffness(_edgeStiffness);
  _softBodyNode->setDampingCoefficient(_dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  size_t nTopPointMasses = _nSlices * (_nRings - 1) + 1;
  size_t nDrumPointMasses = (_nStacks + 1) * _nSlices;
  size_t nTotalMasses = nDrumPointMasses + 2 * nTopPointMasses;

  // Mass per vertices
  double mass = _totalMass / nTotalMasses;

  // Point mass pointer
  PointMass* newPointMass = NULL;

  // Resting positions for each point mass
  float dradius = _radius / static_cast<float>(_nRings);
  float dtheta = DART_2PI / static_cast<float>(_nSlices);

  // -- top
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, 0.5 * _height));
  _softBodyNode->addPointMass(newPointMass);
  for (size_t i = 1; i < _nRings; ++i)
  {
    float z = 0.5;
    float radius = i * dradius;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * radius, y * radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }

  // -- middle
  float dz     = -1.0 / static_cast<float>(_nStacks);
  for (size_t i = 0; i < _nStacks + 1; i++)
  {
    float z = 0.5 + i * dz;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * _radius, y * _radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }

  // -- bottom
  for (size_t i = 1; i < _nRings; ++i)
  {
    float z = -0.5;
    float radius = _radius - i * dradius;

    for (size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      newPointMass = new dynamics::PointMass(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * radius, y * radius, z * _height));
      _softBodyNode->addPointMass(newPointMass);
    }
  }
  newPointMass = new dynamics::PointMass(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(
        Eigen::Vector3d(0.0, 0.0, -0.5 * _height));
  _softBodyNode->addPointMass(newPointMass);

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // A. Drum part

  // a) longitudinal
  // -- top
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(0, i + 1);
  for (size_t i = 0; i < _nRings - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            _nSlices + 1 + (i + 0) * _nSlices + j,
            _nSlices + 1 + (i + 1) * _nSlices + j);
    }
  }
  // -- middle
  for (size_t i = 0; i < _nStacks - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j,
            nTopPointMasses + (i + 1) * _nSlices + j);
    }
  }
  // -- bottom
  for (size_t i = 0; i < _nRings - 1; i++)
  {
    for (size_t j = 0; j < _nSlices; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 0) * _nSlices + j,
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 1) * _nSlices + j);
    }
  }
  for (size_t i = 0; i < _nSlices; i++)
    _softBodyNode->connectPointMasses(nTotalMasses - 1 - i,
                                      nTotalMasses - 1);

  // b) latitudinal
  for (size_t i = 0; i < _nStacks; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + i * _nSlices + j + 0,
            nTopPointMasses + i * _nSlices + j + 1);
    }

    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 0) * _nSlices);
  }
  // -- disk parts
  // TODO(JS): No latitudinal connections for top and bottom disks

  // c) cross (shear)
  // -- drum part
  for (size_t i = 0; i < _nStacks - 2; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 0,
            nTopPointMasses + (i + 1) * _nSlices + j + 1);
      _softBodyNode->connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 1,
            nTopPointMasses + (i + 1) * _nSlices + j + 0);
    }

    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 1) * _nSlices);
    _softBodyNode->connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices,
          nTopPointMasses + (i + 1) * _nSlices + _nSlices - 1);
  }
  // -- disk parts
  // TODO(JS): No cross connections for top and bottom disks

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  size_t nConePointMass = 1;
  meshIdx1 = 0;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  for (size_t i = 0; i < _nRings - 1; ++i)
  {
    for (size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    _softBodyNode->addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));
  }

  // middle
  for (size_t i = 0; i < _nStacks; i++)
  {
    for (size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices;
    meshIdx2 = (i + 1) * _nSlices;
    meshIdx3 = (i + 0) * _nSlices + _nSlices - 1;
    _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + 0;
    meshIdx3 = (i + 1) * _nSlices + _nSlices - 1;
    _softBodyNode->addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));
  }

  // bottom
  for (size_t i = 0; i < _nRings - 1; ++i)
  {
    for (size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      _softBodyNode->addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      _softBodyNode->addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    _softBodyNode->addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    _softBodyNode->addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
  }
  meshIdx1 = 1;
  for (size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 2;
    meshIdx3 = i + 3;
    _softBodyNode->addFace(Eigen::Vector3i(
                             nTotalMasses - meshIdx1,
                             nTotalMasses - meshIdx2,
                             nTotalMasses - meshIdx3));
  }
  meshIdx2 = _nSlices + 1;
  meshIdx3 = 2;
  _softBodyNode->addFace(Eigen::Vector3i(
                           nTotalMasses - meshIdx1,
                           nTotalMasses - meshIdx2,
                           nTotalMasses - meshIdx3));
}

}  // namespace dynamics
}  // namespace dart

