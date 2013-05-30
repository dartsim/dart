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

#include "BodyNode.h"
#include <algorithm>
#include "Shape.h"
#include "Joint.h"
#include "Marker.h"
#include "Dof.h"
#include "Skeleton.h"
#include "Transformation.h"
#include "utils/Misc.h"
#include "math/UtilsMath.h"
#include "renderer/RenderInterface.h"

using namespace std;
using namespace Eigen;

namespace kinematics
{

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode(const char *_name)
    : mSkelIndex(-1),
      mVizShape(NULL),
      mColShape(NULL),
      mParentJoint(NULL),
      mParentNode(NULL),
      mColliding(false),
      mCollidable(true),
      mNumRootTrans(0),
      mMass(0),
      mCOMLocal(0,0,0),
      mSkel(NULL),
      mI(Matrix3d::Zero())
{
    mJointsChild.clear();
    mMarkers.clear();
    mDependentDofs.clear();

    mID = BodyNode::msBodyNodeCount++;

    if (_name == NULL)
        strcpy(mName, "BodyNode");
    else
        strcpy(mName, _name);
}

BodyNode::~BodyNode()
{
    for (unsigned int i = 0; i < mMarkers.size(); ++i)
        delete mMarkers[i];

    mMarkers.clear();

    if(mVizShape && mVizShape == mColShape)
    {
        delete mVizShape;
        mVizShape = NULL;
        mColShape = NULL;
    }

    if(mVizShape)
    {
        delete mVizShape;
        mVizShape = NULL;
    }

    if(mColShape)
    {
        delete mColShape;
        mColShape = NULL;
    }

    mJointsChild.clear();
}

void BodyNode::init()
{
    assert(mSkel);

    if(mVizShape && mI.isZero())
        mI = mVizShape->computeInertia(mMass);

    mT = Matrix4d::Identity();
    mW = Matrix4d::Identity();
    mIc = Matrix3d::Zero();

    const int numLocalDofs = getNumLocalDofs();
    mTq.resize(numLocalDofs, Matrix4d::Zero());
    const int numDepDofs = getNumDependentDofs();
    mWq.resize(numDepDofs, Matrix4d::Zero());

    mJv = MatrixXd::Zero(3, numDepDofs);
    mJw = MatrixXd::Zero(3, numDepDofs);
    mBodyJacobian = MatrixXd::Zero(6, numDepDofs);

    mNumRootTrans = 0;

    for(int i=0; i<mSkel->getNumDofs(); i++)
    {
        if(!(mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATE
             || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEX
             || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEY
             || mSkel->getDof(i)->getTrans()->getType()==Transformation::T_TRANSLATEZ))
        {
            mNumRootTrans = i;
            break;
        }
    }
}

void BodyNode::updateTransform()
{
    mT = mParentJoint->getLocalTransform();

    if (mParentNode)
        mW.noalias() = mParentNode->mW * mT;
    else
        mW = mT;

    // TODO: This will be removed once we use mInertia instead of mI and mIc.
    // update the inertia matrix
    Matrix3d R = mW.topLeftCorner<3,3>();
    mIc.noalias() = R*mI*R.transpose();
}

void BodyNode::updateVelocity()
{
    // Generalized body velocity from parent link.
    if (mParentNode)
        mBodyVelocity = dart_math::InvAd(mT, mParentNode->mBodyVelocity);
    else
        mBodyVelocity.setZero();

    // Generalized body velocity from parent joint.
    mBodyVelocity += mParentJoint->getLocalJacobian() * mParentJoint->get_dq();
}

void BodyNode::updateAcceleration()
{
    mBodyAcceleration.setZero();

    // Generalized body velocity from parent link.
    if (mParentNode)
        mBodyVelocity = dart_math::InvAd(mT, mParentNode->mBodyAcceleration);
    else
        mBodyVelocity.setZero();

    // Generalized body velocity from parent joint.
    mBodyVelocity += mParentJoint->getLocalJacobian() * mParentJoint->get_ddq();
}

void BodyNode::updateFirstDerivatives()
{
    const int numLocalDofs = getNumLocalDofs();
    const int numParentDofs = getNumDependentDofs()-numLocalDofs;

    // Update Local Derivatives
    for(int i = 0; i < numLocalDofs; i++)
        mTq[i] = getLocalDeriv(getDof(i));

    // Update World Derivatives
    // parent dofs
    for (int i = 0; i < numParentDofs; i++)
    {
        assert(mParentNode);    // should always have a parent if enters this for loop
        if(i<mNumRootTrans)
            mWq[i] = mParentNode->mWq[i]; // in turn its equal to dT/dqi where T is the translation 4x4 matrix for the first 3 dofs
        else
            mWq[i].noalias() = mParentNode->mWq[i] * mT;
    }
    // local dofs
    for(int i = 0; i < numLocalDofs; i++)
    {
        if(mParentNode)
            mWq[numParentDofs+i].noalias() = mParentNode->mW * mTq[i];
        else
            mWq[i] = mTq[i];
    }

    evalJacobian();
}

void BodyNode::evalJacLin() {
    assert(mJv.rows() == 3 && mJv.cols() == mDependentDofs.size());

    for (unsigned int i = 0; i < mDependentDofs.size(); i++) {
        mJv.col(i) = dart_math::xformHom(mWq[i], mCOMLocal);
    }
}

void BodyNode::evalJacAng() {
    mJw.setZero();
    for (unsigned int i=mNumRootTrans; i<mDependentDofs.size(); i++) {
        Matrix3d omegaSkewSymmetric = mWq[i].topLeftCorner<3,3>() * mW.topLeftCorner<3,3>().transpose();  // wikipedia calls this the angular velocity tensor
        mJw.col(i) = dart_math::fromSkewSymmetric(omegaSkewSymmetric);
    }
}

void BodyNode::evalJacobian()
{
    /*--------------------------------------------------------------------------
    J             : (6xn matrix) Body Jacobian of this link
    T_{i,j}       : (4x4 matrix) Transformation matrix from i-th frame to
                    j-th frame
    S_i           : (6x1 vector) Local Jacobian of i-th joint
    Ad(T_{i,j},S) : (6x6 matrix) Adjoint mapping

    J = | Ad(T_{n,1}, S_1) Ad(T_{n,2}, S_2) ... Ad(T_{n,n-1}, S_{n-1}) S_n |
    --------------------------------------------------------------------------*/

    const int numLocalDofs = getNumLocalDofs();
    const int numDepDofs = getNumDependentDofs();

    //MatrixXd J = MatrixXd::Zero(6,numDepDofs);
    assert(mBodyJacobian.rows() == 6);
    assert(mBodyJacobian.cols() == getNumDependentDofs());

    // The parent link has:
    //   parentJacobian = | Ad(T_{n-1,1}, S_1) Ad(T_{n-1,2}, S_2) ... S_{n-1} |.
    // The parent joint has:
    //   localJacobian = S_n.
    // We bind these two Jacobian as:
    // J = | Ad(mT^{-1},parentJacobian) localJacobian |
    //   = | InvAd(mT,parentJacobian) localJacobian |

    // Bind InvAd(mT,parentJacobian) first.
    // J = | InvAd(mT,parentJacobian) 0 |
    if (mParentNode != NULL)
    {
        Eigen::MatrixXd parentJacobian = mParentNode->getBodyJacobian();
        for (int i = 0; i < parentJacobian.cols(); ++i)
            mBodyJacobian.col(i) = dart_math::InvAd(mT, parentJacobian.col(i));
    }

    // Bind localJacobian.
    // J = | InvAd(mT,parentJacobian) localJacobian |
    if (numLocalDofs > 0)
        mBodyJacobian.block(0, numDepDofs - numLocalDofs, 6, numLocalDofs)
                = mParentJoint->getLocalJacobian();

    // TODO: below code will be deleted once mJw, mJv are deprecated.
    const Eigen::MatrixXd& Jworld = getWorldJacobian();
    //mJw = Jworld.topLeftCorner(3,numDepDofs);
    //mJv = Jworld.bottomLeftCorner(3,numDepDofs);

    evalJacAng();
    evalJacLin();
}

Vector3d BodyNode::evalWorldPos(const Vector3d& _lp) const
{
    return dart_math::xformHom(mW, _lp);
}

Matrix4d BodyNode::getLocalDeriv(Dof* _q) const
{
    return mParentJoint->getDeriv(_q);
}

void BodyNode::setDependDofList()
{
    mDependentDofs.clear();
    if (mParentNode != NULL)
    {
        mDependentDofs.insert(mDependentDofs.end(),
                              mParentNode->mDependentDofs.begin(),
                              mParentNode->mDependentDofs.end());
    }

    for (int i = 0; i < getNumLocalDofs(); i++)
    {
        int dofID = getDof(i)->getSkelIndex();
        mDependentDofs.push_back(dofID);
    }

#if _DEBUG
    for (int i = 0; i < (int)mDependentDofs.size() - 1; i++) {
        int now = mDependentDofs[i];
        int next = mDependentDofs[i + 1];
        if (now > next) {
            cerr << "Array not sorted!!!" << endl;
            exit(0);
        }
    }
#endif
}

bool BodyNode::dependsOn(int _dofIndex) const
{
    return binary_search(mDependentDofs.begin(),
                         mDependentDofs.end(),
                         _dofIndex);
}

void BodyNode::draw(renderer::RenderInterface* _ri,
                    const Vector4d& _color,
                    bool _useDefaultColor,
                    int _depth) const
{
    if (!_ri)
        return;

    _ri->pushMatrix();

    // render the self geometry
    for (int i = 0; i < mParentJoint->getNumTransforms(); i++)
    {
        mParentJoint->getTransform(i)->applyGLTransform(_ri);
    }

    if (mVizShape != NULL)
    {
        _ri->pushName((unsigned)mID);
        _ri->pushMatrix();
        mVizShape->draw(_ri, _color, _useDefaultColor);
        _ri->popMatrix();
        _ri->popName();
    }

    // render the subtree
    for (unsigned int i = 0; i < mJointsChild.size(); i++)
    {
        mJointsChild[i]->getChildNode()->draw(_ri, _color, _useDefaultColor);
    }

    _ri->popMatrix();

}

void BodyNode::drawMarkers(renderer::RenderInterface* _ri,
                           const Vector4d& _color,
                           bool _useDefaultColor) const
{
    if (!_ri)
        return;

    _ri->pushMatrix();

    for (int i = 0; i < mParentJoint->getNumTransforms(); i++)
        mParentJoint->getTransform(i)->applyGLTransform(_ri);

    // render the corresponding mMarkerss
    for (unsigned int i = 0; i < mMarkers.size(); i++)
        mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);

    for (unsigned int i = 0; i < mJointsChild.size(); i++)
        mJointsChild[i]->getChildNode()->drawMarkers(_ri,_color, _useDefaultColor);

    _ri->popMatrix();
}

Eigen::Matrix4d BodyNode::getMassTensor() const
{
    // compute the half trace of mat = row*integral((x*x+y*y+z*z)dxdydz) = sum
    // of moment of inertia along all 3 axes
    const double halftrace = 0.5 * mI.trace();
    Eigen::Matrix4d massTensor;

    massTensor << halftrace * Matrix3d::Identity() - mI, Vector3d::Zero(),
                  RowVector3d::Zero()                  , mMass;

    return massTensor;
}

void BodyNode::setParentJoint(Joint *_p)
{
    mParentJoint = _p;
    mParentNode = _p->getParentNode();
}

BodyNode* BodyNode::getChildNode(int _idx) const
{
    return mJointsChild[_idx]->getChildNode();
}

int BodyNode::getNumLocalDofs() const
{
    return mParentJoint->getNumDofs();
}

Dof* BodyNode::getDof(int _idx) const
{
    return mParentJoint->getDof(_idx);
}

bool BodyNode::isPresent(const Dof* _q)
{
    return mParentJoint->isPresent(_q);
}

const Matrix4d& BodyNode::getDerivLocalTransform(int index) const
{
    return mTq[index];
}

const Matrix4d& BodyNode::getDerivWorldTransform(int index) const
{
    return mWq[index];
}

const MatrixXd& BodyNode::getJacobianLinear() const
{
    return mJv;
}

const MatrixXd& BodyNode::getJacobianAngular() const
{
    return mJw;
}

const Eigen::MatrixXd& BodyNode::getBodyJacobian() const
{
    return mBodyJacobian;
}

Eigen::MatrixXd BodyNode::getWorldJacobian() const
{
    const Eigen::MatrixXd& bodyJacobian = getBodyJacobian();
    Eigen::MatrixXd worldJacobian = Eigen::MatrixXd::Zero(mBodyJacobian.rows(), mBodyJacobian.cols());

    Matrix4d X = Matrix4d::Identity();
    X.topLeftCorner<3,3>() = mW.topLeftCorner<3,3>();

    for (int i = 0; i < worldJacobian.cols(); ++i)
        worldJacobian.col(i) = dart_math::Ad(X, bodyJacobian.col(i));

    return worldJacobian;
}

void BodyNode::evalVelocity(const VectorXd &_qDotSkel)
{
    mVel.setZero();

    for(int i = 0; i < getNumDependentDofs(); i++)
        mVel += mJv.col(i)*_qDotSkel[mDependentDofs[i]];
}

void BodyNode::evalOmega(const VectorXd &_qDotSkel)
{
    mOmega.setZero();

    for(int i = mNumRootTrans; i < getNumDependentDofs(); i++)
        mOmega += mJw.col(i)*_qDotSkel[mDependentDofs[i]];
}

dart_math::Vector6d BodyNode::getWorldVelocity() const
{
    dart_math::Vector6d worldVelcocity;

    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.topLeftCorner<3,3>() = mW.topLeftCorner<3,3>();
    worldVelcocity = dart_math::Ad(X, mBodyVelocity);

    return worldVelcocity;
}

dart_math::Vector6d BodyNode::getWorldAcceleration() const
{
    dart_math::Vector6d worldAcceleration;

    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.topLeftCorner<3,3>() = mW.topLeftCorner<3,3>();
    worldAcceleration = dart_math::Ad(X, mBodyAcceleration);

    return worldAcceleration;
}

} // namespace kinematics

