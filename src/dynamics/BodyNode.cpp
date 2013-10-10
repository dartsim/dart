/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

#include "dynamics/BodyNode.h"

#include <iostream>
#include <algorithm>

#include "common/Console.h"
#include "math/Helpers.h"
#include "renderer/RenderInterface.h"
#include "dynamics/Joint.h"
#include "dynamics/Shape.h"
#include "dynamics/Skeleton.h"
#include "dynamics/Marker.h"

namespace dart {
namespace dynamics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode(const std::string& _name)
    : mSkelIndex(-1),
      mName(_name),
      mIsCollidable(true),
      mIsColliding(false),
      mSkeleton(NULL),
      mParentJoint(NULL),
      mParentBodyNode(NULL),
      mChildBodyNodes(std::vector<BodyNode*>(0)),
      mGravityMode(true),
      mCenterOfMass(Eigen::Vector3d::Zero()),
      mMass(1.0),
      mIxx(1.0),
      mIyy(1.0),
      mIzz(1.0),
      mIxy(0.0),
      mIxz(0.0),
      mIyz(0.0),
      mI(Eigen::Matrix6d::Identity()),
      mW(Eigen::Isometry3d::Identity()),
      mV(Eigen::Vector6d::Zero()),
      mEta(Eigen::Vector6d::Zero()),
      mdV(Eigen::Vector6d::Zero()),
      mF(Eigen::Vector6d::Zero()),
      mFext(Eigen::Vector6d::Zero()),
      mFgravity(Eigen::Vector6d::Zero()),
      mB(Eigen::Vector6d::Zero()),
      mBeta(Eigen::Vector6d::Zero()),
      mID(BodyNode::msBodyNodeCount++)
{
}

BodyNode::~BodyNode()
{
    for (std::vector<Shape*>::const_iterator it = mVizShapes.begin();
         it != mVizShapes.end(); ++it)
        delete (*it);

    for (std::vector<Shape*>::const_iterator itColShape = mColShapes.begin();
         itColShape != mColShapes.end(); ++itColShape)
        if (mVizShapes.end() == find(mVizShapes.begin(), mVizShapes.end(), *itColShape))
            delete (*itColShape);

    for (std::vector<Marker*>::const_iterator it = mMarkers.begin();
         it != mMarkers.end(); ++it)
        delete (*it);

    if (mParentJoint)
        delete mParentJoint;
}

void BodyNode::setName(const std::string& _name)
{
    mName = _name;
}

const std::string& BodyNode::getName() const
{
    return mName;
}

void BodyNode::setGravityMode(bool _gravityMode)
{
    mGravityMode = _gravityMode;
}

bool BodyNode::getGravityMode() const
{
    return mGravityMode;
}

bool BodyNode::isCollidable() const
{
    return mIsCollidable;
}

void BodyNode::setCollidable(bool _isCollidable)
{
    mIsCollidable = _isCollidable;
}

void BodyNode::setMass(double _mass)
{
    assert(_mass >= 0.0 && "Negative mass is not allowable.");
    mMass = _mass;
    _updateGeralizedInertia();
}

double BodyNode::getMass() const
{
    return mMass;
}

BodyNode*BodyNode::getParentBodyNode() const
{
    return mParentBodyNode;
}

void BodyNode::addChildBodyNode(BodyNode* _body)
{
    assert(_body != NULL);

    mChildBodyNodes.push_back(_body);
    _body->mParentBodyNode = this;
}

BodyNode* BodyNode::getChildBodyNode(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodyNodes.size());

    return mChildBodyNodes[_idx];
}

int BodyNode::getNumChildBodyNodes() const
{
    return mChildBodyNodes.size();
}

void BodyNode::addMarker(Marker* _marker)
{
    mMarkers.push_back(_marker);
}

int BodyNode::getNumMarkers() const
{
    return mMarkers.size();
}

Marker* BodyNode::getMarker(int _idx) const
{
    return mMarkers[_idx];
}

bool BodyNode::dependsOn(int _genCoordIndex) const
{
    return binary_search(mDependentDofIndexes.begin(),
                         mDependentDofIndexes.end(),
                         _genCoordIndex);
}

int BodyNode::getNumDependentDofs() const
{
    return mDependentDofIndexes.size();
}

int BodyNode::getDependentDof(int _arrayIndex) const
{
    return mDependentDofIndexes[_arrayIndex];
}

const Eigen::Isometry3d& BodyNode::getWorldTransform() const
{
    return mW;
}

const Eigen::Vector6d& BodyNode::getBodyVelocity() const
{
    return mV;
}

Eigen::Vector6d BodyNode::getWorldVelocity(const Eigen::Vector3d& _offset) const
{
    Eigen::Isometry3d T = mW;
    T.translation() = -_offset;
    return math::AdT(T, mV);
}

const Eigen::Vector6d&BodyNode::getBodyAcceleration() const
{
    return mdV;
}

Eigen::Vector6d BodyNode::getWorldAcceleration(
        const Eigen::Vector3d& _offset) const
{
    Eigen::Isometry3d T = mW;
    T.translation() = -_offset;
    return math::AdT(T, mdV);
}

const math::Jacobian&BodyNode::getBodyJacobian() const
{
    return mBodyJacobian;
}

math::Jacobian BodyNode::getWorldJacobian(const Eigen::Vector3d& _offset) const
{
    Eigen::Isometry3d T = mW;
    T.translation() = -_offset;
    return math::AdTJac(T, mBodyJacobian);
}

const math::Jacobian& BodyNode::getBodyJacobianTimeDeriv() const
{
    return mBodyJacobianTimeDeriv;
}

math::Jacobian BodyNode::getWorldJacobianTimeDeriv(
        const Eigen::Vector3d& _offset) const
{
    Eigen::Isometry3d T = mW;
    T.translation() = -_offset;
    return math::AdTJac(T, mBodyJacobianTimeDeriv);
}

void BodyNode::setColliding(bool _isColliding)
{
    mIsColliding = _isColliding;
}

bool BodyNode::isColliding()
{
    return mIsColliding;
}

void BodyNode::init(Skeleton* _skeleton, int _skeletonIndex)
{
    assert(_skeleton);

    mSkeleton = _skeleton;
    mSkelIndex = _skeletonIndex;
    mParentJoint->mSkelIndex = _skeletonIndex;

    // fill list of generalized coordinates this node depends on
    if (mParentBodyNode)
        mDependentDofIndexes = mParentBodyNode->mDependentDofIndexes;
    else
        mDependentDofIndexes.clear();
    for (int i = 0; i < mParentJoint->getNumGenCoords(); i++)
        mDependentDofIndexes.push_back(mParentJoint->getGenCoord(i)->getSkeletonIndex());

#ifndef NDEBUG
    for (int i = 0; i < (int)mDependentDofIndexes.size() - 1; i++)
    {
        for (int j = i + 1; j < mDependentDofIndexes.size(); j++)
            if (mDependentDofIndexes[i] == mDependentDofIndexes[j])
            {
                dterr << "Skeleton ID of Generalized coordinates is duplicated."
                      << std::endl;
            }
    }
#endif

    const int numDepDofs = getNumDependentDofs();
    mBodyJacobian      = math::Jacobian::Zero(6,numDepDofs);
    mBodyJacobianTimeDeriv = math::Jacobian::Zero(6,numDepDofs);
    mM                 = Eigen::MatrixXd::Zero(numDepDofs, numDepDofs);
}

void BodyNode::draw(renderer::RenderInterface* _ri,
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
    for(int i = 0; i < mVizShapes.size(); i++)
    {
        _ri->pushMatrix();
        mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
        _ri->popMatrix();
    }
    _ri->popName();

    // render the subtree
    for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
    {
        mChildBodyNodes[i]->draw(_ri, _color, _useDefaultColor);
    }

    _ri->popMatrix();
}

void BodyNode::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const
{
    if (!_ri)
        return;

    _ri->pushMatrix();

    mParentJoint->applyGLTransform(_ri);

    // render the corresponding mMarkerss
    for (unsigned int i = 0; i < mMarkers.size(); i++)
        mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);

    for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
        mChildBodyNodes[i]->drawMarkers(_ri,_color, _useDefaultColor);

    _ri->popMatrix();
}

void BodyNode::updateTransform()
{
    mParentJoint->updateTransform();

    if (mParentBodyNode)
    {
        mW = mParentBodyNode->getWorldTransform()
             * mParentJoint->getLocalTransform();
    }
    else
    {
        mW = mParentJoint->getLocalTransform();
    }

    assert(math::verifyTransform(mW));
}

void BodyNode::updateVelocity(bool _updateJacobian)
{
    //--------------------------------------------------------------------------
    // Body velocity update
    //
    // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
    //--------------------------------------------------------------------------

    mParentJoint->updateJacobian();

    if (mParentJoint->getNumGenCoords() > 0)
    {
        if (mParentBodyNode)
        {
            mV = math::AdInvT(mParentJoint->getLocalTransform(),
                              mParentBodyNode->getBodyVelocity()) +
                    mParentJoint->getLocalJacobian() * mParentJoint->get_dq();
        }
        else
        {
            mV = mParentJoint->getLocalJacobian() * mParentJoint->get_dq();
        }
    }

    assert(!math::isNan(mV));

    if (_updateJacobian == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian update
    //
    // J = | J1 J2 ... Jn |
    //   = | Ad(T(i,i-1), J_parent) J_local |
    //
    //   J_parent: (6 x parentDOF)
    //    J_local: (6 x localDOF)
    //         Ji: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = mParentJoint->getNumGenCoords();
    const int numParentDOFs = getNumDependentDofs()-numLocalDOFs;

    // Parent Jacobian
    if (mParentBodyNode != NULL)
    {
        assert(mParentBodyNode->mBodyJacobian.cols() + mParentJoint->getNumGenCoords()
               == mBodyJacobian.cols());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            mBodyJacobian.col(i) = math::AdInvT(
                                       mParentJoint->getLocalTransform(),
                                       mParentBodyNode->mBodyJacobian.col(i));
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobian.col(numParentDOFs + i) =
                mParentJoint->getLocalJacobian().col(i);
    }
}

void BodyNode::updateEta()
{
    mParentJoint->updateJacobianTimeDeriv();

    if (mParentJoint->getNumGenCoords() > 0)
    {
        mEta = math::ad(mV, mParentJoint->getLocalJacobian() *
                            mParentJoint->get_dq()) +
                            mParentJoint->getLocalJacobianTimeDeriv() *
                            mParentJoint->get_dq();

        assert(!math::isNan(mEta));
    }
}

void BodyNode::updateAcceleration(bool _updateJacobianDeriv)
{
    // dV(i) = Ad(T(i, i-1), dV(i-1))
    //         + ad(V(i), S * dq) + dS * dq
    //         + S * ddq
    //       = Ad(T(i, i-1), dV(i-1))
    //         + eta
    //         + S * ddq

    if (mParentJoint->getNumGenCoords() > 0)
    {
        if (mParentBodyNode)
        {
            mdV = math::AdInvT(mParentJoint->getLocalTransform(),
                               mParentBodyNode->getBodyAcceleration()) +
                               mEta + mParentJoint->getLocalJacobian() *
                               mParentJoint->get_ddq();
        }
        else
        {
            mdV = mEta +
                  mParentJoint->getLocalJacobian() * mParentJoint->get_ddq();
        }
    }

    assert(!math::isNan(mdV));

    if (_updateJacobianDeriv == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian first derivative update
    //
    // dJ = | dJ1 dJ2 ... dJn |
    //   = | Ad(T(i,i-1), dJ_parent) dJ_local |
    //
    //   dJ_parent: (6 x parentDOF)
    //    dJ_local: (6 x localDOF)
    //         dJi: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = mParentJoint->getNumGenCoords();
    const int numParentDOFs = getNumDependentDofs() - numLocalDOFs;

    // Parent Jacobian
    if (mParentBodyNode != NULL)
    {
        assert(mParentBodyNode->mBodyJacobianTimeDeriv.cols() + mParentJoint->getNumGenCoords()
               == mBodyJacobianTimeDeriv.cols());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            Eigen::Vector6d dJi = math::AdInvT(mParentJoint->getLocalTransform(),
                                         mParentBodyNode->mBodyJacobianTimeDeriv.col(i));
            mBodyJacobianTimeDeriv.col(i) = dJi;
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobianTimeDeriv.col(numParentDOFs + i) =
                mParentJoint->getLocalJacobianTimeDeriv().col(i);
    }
}

void BodyNode::setInertia(double _Ixx, double _Iyy, double _Izz,
                          double _Ixy, double _Ixz, double _Iyz)
{
    assert(_Ixx >= 0.0);
    assert(_Iyy >= 0.0);
    assert(_Izz >= 0.0);

    mIxx = _Ixx;
    mIyy = _Iyy;
    mIzz = _Izz;

    mIxy = _Ixy;
    mIxz = _Ixz;
    mIyz = _Iyz;

    _updateGeralizedInertia();
}

void BodyNode::setLocalCOM(const Eigen::Vector3d& _com)
{
    mCenterOfMass = _com;

    _updateGeralizedInertia();
}

const Eigen::Vector3d& BodyNode::getLocalCOM() const
{
    return mCenterOfMass;
}

Eigen::Vector3d BodyNode::getWorldCOM() const
{
    return mW.linear() * mCenterOfMass;
}

Eigen::Matrix6d BodyNode::getInertia() const
{
    return mI;
}

int BodyNode::getSkeletonIndex() const
{
    return mSkelIndex;
}

void BodyNode::addVisualizationShape(Shape* _p)
{
    mVizShapes.push_back(_p);
}

int BodyNode::getNumVisualizationShapes() const
{
    return mVizShapes.size();
}

Shape*BodyNode::getVisualizationShape(int _idx) const
{
    return mVizShapes[_idx];
}

void BodyNode::addCollisionShape(Shape* _p)
{
    mColShapes.push_back(_p);
}

int BodyNode::getNumCollisionShapes() const
{
    return mColShapes.size();
}

Shape*BodyNode::getCollisionShape(int _idx) const
{
    return mColShapes[_idx];
}

Skeleton*BodyNode::getSkeleton() const
{
    return mSkeleton;
}

void BodyNode::setParentJoint(Joint* _joint)
{
    mParentJoint = _joint;
}

Joint*BodyNode::getParentJoint() const
{
    return mParentJoint;
}

void BodyNode::addExtForce(const Eigen::Vector3d& _offset,
                           const Eigen::Vector3d& _force,
                           bool _isOffsetLocal, bool _isForceLocal)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Vector6d F = Eigen::Vector6d::Zero();

    if (_isOffsetLocal)
        T.translation() = _offset;
    else
        T.translation() = getWorldTransform().inverse() * _offset;

    if (_isForceLocal)
        F.tail<3>() = _force;
    else
        F.tail<3>() = mW.linear().transpose() * _force;

    mFext += math::dAdInvT(T, F);
}

void BodyNode::setExtForce(const Eigen::Vector3d& _offset,
                           const Eigen::Vector3d& _force,
                           bool _isOffsetLocal, bool _isForceLocal)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Vector6d F = Eigen::Vector6d::Zero();

    if (_isOffsetLocal)
        T.translation() = _offset;
    else
        T.translation() = getWorldTransform().inverse() * _offset;

    if (_isForceLocal)
        F.tail<3>() = _force;
    else
        F.tail<3>() = mW.linear().transpose() * _force;

    mFext = math::dAdInvT(T, F);
}

void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    if (_isLocal)
        mFext.head<3>() += _torque;
    else
        mFext.head<3>() += mW.linear() * _torque;
}

void BodyNode::setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    if (_isLocal)
        mFext.head<3>() = _torque;
    else
        mFext.head<3>() = mW.linear() * _torque;
}

const Eigen::Vector6d& BodyNode::getExternalForceLocal() const
{
    return mFext;
}

Eigen::Vector6d BodyNode::getExternalForceGlobal() const
{
    return math::dAdInvT(mW, mFext);
}

void BodyNode::addContactForce(const Eigen::Vector6d& _contactForce)
{
    mContactForces.push_back(_contactForce);
}

int BodyNode::getNumContactForces() const
{
    return mContactForces.size();
}

const Eigen::Vector6d& BodyNode::getContactForce(int _idx)
{
    assert(0 <= _idx && _idx < mContactForces.size());

    return mContactForces[_idx];
}

void BodyNode::clearContactForces()
{
    mContactForces.clear();
}

const Eigen::Vector6d& BodyNode::getBodyForce() const
{
    return mF;
}

double BodyNode::getKineticEnergy() const
{
    return 0.5 * mV.dot(mI * mV);
}

Eigen::Vector3d BodyNode::evalLinMomentum() const
{
    return (mI * mV).tail<3>();
}

Eigen::Vector3d BodyNode::evalAngMomentum(Eigen::Vector3d _pivot)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = _pivot;
    return math::dAdT(T, mI * mV).head<3>();
}

void BodyNode::updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces)
{
    if (mGravityMode == true)
        mFgravity = mI * math::AdInvRLinear(mW, _gravity);
    else
        mFgravity.setZero();

    mF = mI * mdV;                // Inertial force
    if (_withExternalForces)
        mF -= mFext;              // External force
    mF -= mFgravity;              // Gravity force
    mF -= math::dad(mV, mI * mV); // Coriolis force

    for (std::vector<BodyNode*>::iterator iChildBody = mChildBodyNodes.begin();
         iChildBody != mChildBodyNodes.end();
         ++iChildBody)
    {
        dynamics::Joint* childJoint = (*iChildBody)->getParentJoint();
        assert(childJoint != NULL);

        mF += math::dAdInvT(childJoint->getLocalTransform(),
                            (*iChildBody)->getBodyForce());
    }

    assert(!math::isNan(mF));
}

void BodyNode::updateGeneralizedForce(bool _withDampingForces)
{
    assert(mParentJoint != NULL);

    const math::Jacobian& J = mParentJoint->getLocalJacobian();

//    if (_withDampingForces)
//        mF -= mFDamp;

    mParentJoint->set_tau(J.transpose()*mF);
}

void BodyNode::updateArticulatedInertia()
{
    mAI = mI;

    std::vector<BodyNode*>::iterator it;
    for (it = mChildBodyNodes.begin(); it != mChildBodyNodes.end(); ++it)
    {
        mAI += math::transformInertia(
                    (*it)->getParentJoint()->getLocalTransform().inverse(),
                    (*it)->mPi);
    }
}

void BodyNode::updateBiasForce(const Eigen::Vector3d& _gravity)
{
    if (mGravityMode == true)
        mFgravity = mI * math::AdInvRLinear(mW, Eigen::Vector3d(_gravity));
    else
        mFgravity.setZero();

    mB = -math::dad(mV, mI*mV) - mFext - mFgravity;

    for (int i = 0; i < mContactForces.size(); ++i)
        mB -= mContactForces[i];

    std::vector<BodyNode*>::iterator it;
    for (it = mChildBodyNodes.begin(); it != mChildBodyNodes.end(); ++it)
        mB += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                            (*it)->mBeta);

    assert(!math::isNan(mB));
}

void BodyNode::updatePsi()
{
    assert(mParentJoint != NULL);

    //int n = mParentJoint->getNumGenCoords();
    //mAI_S = Eigen::MatrixXd::Zero(6, n);
    //mPsi = Eigen::MatrixXd::Zero(n, n);

    mAI_S.noalias() = mAI * mParentJoint->getLocalJacobian();
    mPsi = (mParentJoint->getLocalJacobian().transpose() * mAI_S).inverse();
}

void BodyNode::updatePi()
{
    mPi            = mAI;
    mPi.noalias() -= mAI_S*mPsi*mAI_S.transpose();
}

void BodyNode::updateBeta()
{
    mAlpha           = mParentJoint->get_tau();

    if (mParentJoint->getNumGenCoords() > 0)
    {
        mAlpha          += mParentJoint->getDampingForces();
        Eigen::VectorXd Fc = Eigen::VectorXd::Zero(mParentJoint->getNumGenCoords());
                for (int i = 0; i < mParentJoint->getNumGenCoords(); i++)
                    Fc(i) = mSkeleton->getConstraintForces()[mParentJoint->getGenCoord(i)->getSkeletonIndex()];
                mAlpha += Fc;
    }

    mAlpha          -= mParentJoint->getLocalJacobian().transpose() *
                       (mAI*mEta + mB);
    mBeta            = mB;
    if (mParentJoint->getNumGenCoords() > 0)
        mBeta += mAI*(mEta + mParentJoint->getLocalJacobian() * mPsi * mAlpha);
    else
        mBeta += mAI*mEta;

    assert(!math::isNan(mBeta));
}

void BodyNode::update_ddq()
{
    Eigen::VectorXd ddq;
    if (mParentBodyNode)
    {
        ddq.noalias() = mPsi*
                        (mAlpha -
                         mParentJoint->getLocalJacobian().transpose() * mAI *
                         math::AdInvT(mParentJoint->getLocalTransform(),
                                      mParentBodyNode->getBodyAcceleration())
                         );
    }
    else
    {
        ddq.noalias() = mPsi*mAlpha;
    }

    mParentJoint->set_ddq(ddq);
}

void BodyNode::update_F_fs()
{
    mF.noalias() = mAI*mdV;
    mF          += mB;

    assert(!math::isNan(mF));
}

void BodyNode::updateDampingForce()
{
    dterr << "Not implemented.\n";
}

void BodyNode::updateMassMatrix()
{
    mM.triangularView<Eigen::Upper>() = mBodyJacobian.transpose() *
                                        mI *
                                        mBodyJacobian;
    mM.triangularView<Eigen::StrictlyLower>() = mM.transpose();
}

void BodyNode::aggregateExternalForces(Eigen::VectorXd& _extForce)
{
    assert(mParentJoint != NULL);

    Eigen::VectorXd localForce = mBodyJacobian.transpose() * mFext;

    for(int i = 0; i < getNumDependentDofs(); i++)
        _extForce(mDependentDofIndexes[i]) += localForce(i);
}

void BodyNode::aggregateMass(Eigen::MatrixXd& _M)
{
    for(int i = 0; i < getNumDependentDofs(); i++)
        for(int j = 0; j < getNumDependentDofs(); j++)
            _M(mDependentDofIndexes[i], mDependentDofIndexes[j]) += mM(i, j);
}

void BodyNode::_updateGeralizedInertia()
{
    // G = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |

    // m * r
    double mr0 = mMass * mCenterOfMass[0];
    double mr1 = mMass * mCenterOfMass[1];
    double mr2 = mMass * mCenterOfMass[2];

    // m * [r] * [r]
    double mr0r0 = mr0 * mCenterOfMass[0];
    double mr1r1 = mr1 * mCenterOfMass[1];
    double mr2r2 = mr2 * mCenterOfMass[2];
    double mr0r1 = mr0 * mCenterOfMass[1];
    double mr1r2 = mr1 * mCenterOfMass[2];
    double mr2r0 = mr2 * mCenterOfMass[0];

    mI(0,0) =  mIxx + mr1r1 + mr2r2;   mI(0,1) =  mIxy - mr0r1;           mI(0,2) =  mIxz - mr2r0;           assert(mI(0,3) == 0.0);   mI(0,4) = -mr2;           mI(0,5) =  mr1;
                                       mI(1,1) =  mIyy + mr2r2 + mr0r0;   mI(1,2) =  mIyz - mr1r2;           mI(1,3) =  mr2;           assert(mI(1,4) == 0.0);   mI(1,5) = -mr0;
                                                                          mI(2,2) =  mIzz + mr0r0 + mr1r1;   mI(2,3) = -mr1;           mI(2,4) =  mr0;           assert(mI(2,5) == 0.0);
                                                                                                             mI(3,3) =  mMass;         assert(mI(3,4) == 0.0);   assert(mI(3,5) == 0.0);
                                                                                                                                       mI(4,4) =  mMass;         assert(mI(4,5) == 0.0);
                                                                                                                                                                 mI(5,5) =  mMass;

    mI.triangularView<Eigen::StrictlyLower>() = mI.transpose();
}

void BodyNode::clearExternalForces()
{
    mFext.setZero();
    mContactForces.clear();
}

} // namespace dynamics
} // namespace dart

