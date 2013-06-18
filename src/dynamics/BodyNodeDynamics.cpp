/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#include "BodyNodeDynamics.h"
#include "kinematics/Joint.h"
#include "kinematics/Shape.h"
#include "kinematics/Transformation.h"
//#include "utils/UtilsMath.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace kinematics;

namespace dynamics
{

BodyNodeDynamics::BodyNodeDynamics(const char* const _name)
    : BodyNode(_name),
      mJwJoint(MatrixXd::Zero(3,0)),
      mJwDotJoint(MatrixXd::Zero(3,0)),
      mVelBody(Vector3d::Zero()),
      mVelDotBody(Vector3d::Zero()),
      mOmegaBody(Vector3d::Zero()),
      mOmegaDotBody(Vector3d::Zero()),
      mExtForceBody(Vector3d::Zero()),
      mExtTorqueBody(Vector3d::Zero()),
      mInitializedInvDyn(false),
      mInitializedNonRecursiveDyn(false),
      mGravityMode(true)
{
}

BodyNodeDynamics::~BodyNodeDynamics()
{
}

void BodyNodeDynamics::initInverseDynamics()
{
    if(mInitializedInvDyn) return;
    BodyNode::init();

    mM = MatrixXd::Zero(getNumDependentDofs(), getNumDependentDofs());
    mJwJoint = MatrixXd::Zero(3, getNumDependentDofs());
    mJwDotJoint = MatrixXd::Zero(3, getNumDependentDofs());
    mVelBody.setZero();
    mOmegaBody.setZero();
    mVelDotBody.setZero();
    mOmegaDotBody.setZero();
    mForceJointBody.setZero();
    mTorqueJointBody.setZero();
    // mJv, mJw, and mIc needed for the mass matrix are already inited and computed by the kinematics::BodyNode

    mInitializedInvDyn = true;
}

void BodyNodeDynamics::initDynamics()
{
    if(mInitializedNonRecursiveDyn) return;
    BodyNode::init();

    mVel.setZero();
    mOmega.setZero();
    mM = MatrixXd::Zero(getNumDependentDofs(), getNumDependentDofs());
    mC = MatrixXd::Zero(getNumDependentDofs(), getNumDependentDofs());
    mCvec = VectorXd::Zero(getNumDependentDofs());
    mG = VectorXd::Zero(getNumDependentDofs());
    mFext = VectorXd::Zero(getNumDependentDofs());
    mTqq.resize(getNumLocalDofs());
    for(int i=0; i<getNumLocalDofs(); i++) mTqq[i].resize(getNumLocalDofs(), Matrix4d::Zero());
    mWqq.resize(getNumDependentDofs());
    for(int i=0; i<getNumDependentDofs(); i++) mWqq[i].resize(getNumDependentDofs(), Matrix4d::Zero());
    mJvq.resize(getNumDependentDofs(), MatrixXd::Zero(3, getNumDependentDofs()));
    mJwq.resize(getNumDependentDofs(), MatrixXd::Zero(3, getNumDependentDofs()));
    mJvDot = MatrixXd::Zero(3, getNumDependentDofs());
    mJwDot = MatrixXd::Zero(3, getNumDependentDofs());

    mInitializedNonRecursiveDyn = true;
}

void BodyNodeDynamics::computeInvDynVelocities(const Vector3d &_gravity,
                                               const VectorXd *_qdot,
                                               const VectorXd *_qdotdot,
                                               bool _computeJacobians)
{
    // TODO: TEMP CODE for Gravity Mode
    Vector3d gravity = _gravity;
    if (mGravityMode == false)
        gravity.setZero();

    // update the local transform mT and the world transform mW
    BodyNode::updateTransform();

    mJwJoint    = MatrixXd::Zero(3, mParentJoint->getNumDofsRot());
    mJwDotJoint = MatrixXd::Zero(3, mParentJoint->getNumDofsRot());

    Vector3d omegaJoint    = Vector3d::Zero();
    Vector3d omegaDotJoint = Vector3d::Zero();

    // Local Rotation matrix transposed
    Matrix3d RjointT = mT.topLeftCorner<3,3>().transpose();

    if(mParentJoint->getJointType() != Joint::J_UNKNOWN &&
       mParentJoint->getJointType() != Joint::J_TRANS)
    {
        // ASSUME: trans dofs before rotation dofs
        VectorXd qDotJoint = _qdot->segment(mParentJoint->getFirstRotDofIndex(),
                                            mParentJoint->getNumDofsRot());
        mParentJoint->computeRotationJac(&mJwJoint, &mJwDotJoint, &qDotJoint);

        // Local angular velocity and angular acceleration
        omegaJoint.noalias() = mJwJoint * qDotJoint;
        omegaDotJoint.noalias() = mJwDotJoint * qDotJoint;
        if(_qdotdot)
        {
            VectorXd qDotDotJoint
                    = _qdotdot->segment(mParentJoint->getFirstRotDofIndex(),
                                        mParentJoint->getNumDofsRot());
            omegaDotJoint.noalias() += mJwJoint * qDotDotJoint;
        }
    }

    BodyNodeDynamics *nodeParent = static_cast<BodyNodeDynamics*>(mParentNode);
    Vector3d cl = mCOMLocal;

    if(nodeParent)
    {
        assert(mParentJoint->getNumDofsTrans() == 0); // assuming no internal translation dofs
        Vector3d clparent = nodeParent->getLocalCOM();
        Vector3d rl = mT.topRightCorner<3,1>();    // translation from parent's origin to self origin

        mOmegaBody.noalias() = RjointT * (nodeParent->mOmegaBody + omegaJoint);
        mOmegaDotBody.noalias()
                = RjointT * (nodeParent->mOmegaDotBody
                             + omegaDotJoint
                             + nodeParent->mOmegaBody.cross(omegaJoint));

        mVelBody = mOmegaBody.cross(cl);
        mVelBody.noalias()
                += RjointT * (nodeParent->mVelBody
                              + nodeParent->mOmegaBody.cross(rl-clparent));

        mVelDotBody = mOmegaDotBody.cross(cl)
                      + mOmegaBody.cross(mOmegaBody.cross(cl));
        mVelDotBody.noalias()
                += RjointT * (nodeParent->mVelDotBody
                              + nodeParent->mOmegaDotBody.cross(rl-clparent)
                              + nodeParent->mOmegaBody.cross(
                                  nodeParent->mOmegaBody.cross(rl-clparent)));
    }
    else
    { // base case: root
        mOmegaBody.noalias()    = RjointT * omegaJoint;
        mOmegaDotBody.noalias() = RjointT * omegaDotJoint;

        mVelBody    = mOmegaBody.cross(cl);
        mVelDotBody = mOmegaDotBody.cross(cl) + mOmegaBody.cross(mOmegaBody.cross(cl));

        // Incorporate gravity as part of acceleration by changing frame to the
        // one accelerating with g.
        // Therefore, real acceleration == W*mVelDotBody + g;
        mVelDotBody.noalias() -= RjointT * gravity;

        // if root has translation DOFs
        if (mParentJoint->getNumDofsTrans() > 0)
        {
            VectorXd qDotTransJoint
                    = _qdot->segment(mParentJoint->getFirstTransDofIndex(),
                                     mParentJoint->getNumDofsTrans());
            mVelBody.noalias() += RjointT * qDotTransJoint;
            if(_qdotdot)
            {
                VectorXd qDotDotTransJoint
                        = _qdotdot->segment(mParentJoint->getFirstTransDofIndex(),
                                            mParentJoint->getNumDofsTrans());
                mVelDotBody.noalias() += RjointT * qDotDotTransJoint;
            }
        }
    }

    // compute Jacobians iteratively
    if (_computeJacobians)
    {
        mJv.setZero();
        mJw.setZero();

        // compute the Angular Jacobian first - use it for Linear jacobian as well
        if(nodeParent)
        {
            mJw.leftCols(nodeParent->getNumDependentDofs()) = nodeParent->mJw;
            mJw.rightCols(getNumLocalDofs()).noalias()
                    = nodeParent->mW.topLeftCorner<3,3>() * mJwJoint;

            Vector3d clparent = nodeParent->getLocalCOM();
            Vector3d rl = mT.topRightCorner<3,1>();    // translation from parent's origin to self origin
            mJv.leftCols(nodeParent->getNumDependentDofs()) = nodeParent->mJv;
            mJv.leftCols(nodeParent->getNumDependentDofs()).noalias()
                    -= dart_math::makeSkewSymmetric(
                           nodeParent->mW.topLeftCorner<3,3>()*(rl-clparent))
                       * nodeParent->mJw;
            mJv.noalias() -= dart_math::makeSkewSymmetric(
                                 mW.topLeftCorner<3,3>()*mCOMLocal)
                             * mJw;
        }
        // base case: root
        else
        {
            mJw.rightCols(mParentJoint->getNumDofsRot()) = mJwJoint;

            mJv.noalias() -= dart_math::makeSkewSymmetric(
                                 mW.topLeftCorner<3,3>()*mCOMLocal)
                             * mJw;
            // if root has translation DOFs
            if(mParentJoint->getNumDofsTrans() > 0)
            {
                // ASSUME - 3 translational dofs
                assert(mParentJoint->getNumDofsTrans() == 3);
                // ASSUME - translation dofs in the beginning
                mJv.leftCols(mParentJoint->getNumDofsTrans())
                        += MatrixXd::Identity(3,
                                              mParentJoint->getNumDofsTrans());
            }
        }
    }
}

void BodyNodeDynamics::computeInvDynForces(const Vector3d& /*_gravity*/,
                                           const VectorXd* /*_qdot*/,
                                           const VectorXd* /*_qdotdot*/,
                                           bool _withExternalForces)
{
    mForceJointBody.setZero();
    mTorqueJointBody.setZero();
    Vector3d cl = mCOMLocal;

    // base case: end effectors
    mForceJointBody = mMass*mVelDotBody;
    mTorqueJointBody = cl.cross(mMass*mVelDotBody)
                       + mOmegaBody.cross(mI*mOmegaBody)
                       + mI*mOmegaDotBody;

    // general case
    for(unsigned int j = 0; j < mJointsChild.size(); j++)
    {
        BodyNodeDynamics *bchild = static_cast<BodyNodeDynamics*>(
                                       mJointsChild[j]->getChildNode());
        Matrix3d Rchild = bchild->mT.topLeftCorner<3,3>();
        Vector3d forceChildNode = Rchild*bchild->mForceJointBody;
        mForceJointBody += forceChildNode;
        Vector3d rlchild = bchild->mT.col(3).head(3);
        mTorqueJointBody += (rlchild).cross(forceChildNode) + Rchild*bchild->mTorqueJointBody;
    }

    if( _withExternalForces )
    {
        int nContacts = mContacts.size();
        for(int i=0; i<nContacts; i++){
            mExtForceBody += mContacts.at(i).second;
            mExtTorqueBody += mContacts.at(i).first.cross(mContacts.at(i).second);
        }
        mForceJointBody -= mExtForceBody;
        mTorqueJointBody -= mExtTorqueBody;
    }// endif compute external forces
}

Matrix4d BodyNodeDynamics::getLocalSecondDeriv(const Dof *_q1,
                                               const Dof *_q2) const
{
    return mParentJoint->getSecondDeriv(_q1, _q2);
}

void BodyNodeDynamics::updateSecondDerivatives()
{
    const int numLocalDofs = getNumLocalDofs();
    const int numParentDofs = getNumDependentDofs()-numLocalDofs;
    BodyNodeDynamics *nodeParentDyn = static_cast<BodyNodeDynamics*>(mParentNode);

    // Update Local Derivatives
    for(int i = 0; i < numLocalDofs; i++)
    {
        if(numParentDofs + i<mNumRootTrans) continue;   // since mTq is a constant
        for(int j=0; j<numLocalDofs; j++)
        {
            if(numParentDofs + j<mNumRootTrans) continue;   // since mTq is a constant
            mTqq.at(i).at(j) = getLocalSecondDeriv(getDof(i), getDof(j));
        }
    }

    // Update World Derivatives
    // parent dofs i
    for (int i = mNumRootTrans; i < numParentDofs; i++)
    {
        assert(nodeParentDyn);    // should always have a parent if enters this for loop
        // parent dofs j
        for(int j = mNumRootTrans; j < numParentDofs; j++)
        {
            mWqq.at(i).at(j) = nodeParentDyn->mWqq.at(i).at(j) * mT;
        }
        // local dofs j
        for(int j = 0; j < numLocalDofs; j++)
        {
            mWqq.at(i).at(numParentDofs + j) = nodeParentDyn->mWq.at(i) * mTq.at(j);
        }
    }
    // local dofs i
    for (int i = 0; i < numLocalDofs; i++)
    {
        if(numParentDofs + i<mNumRootTrans) continue;
        // parent dofs j
        for (int j = mNumRootTrans; j < numParentDofs; j++)
        {
            mWqq.at(numParentDofs + i).at(j) = nodeParentDyn->mWq.at(j) * mTq.at(i);
        }
        // local dofs j
        for (int j = 0; j < numLocalDofs; j++)
        {
            if (numParentDofs + j<mNumRootTrans) continue;
            if (nodeParentDyn) mWqq.at(numParentDofs + i).at(numParentDofs + j) = nodeParentDyn->mW * mTqq.at(i).at(j);
            else mWqq.at(numParentDofs + i).at(numParentDofs + j) = mTqq.at(i).at(j);
        }
    }

    for (int i = 0; i < numParentDofs+numLocalDofs; i++)
    {
        evalJacDerivLin(i, getLocalCOM());
        evalJacDerivAng(i);
    }
}

void BodyNodeDynamics::updateSecondDerivatives(Vector3d _offset)
{
    const int numLocalDofs = getNumLocalDofs();
    const int numParentDofs = getNumDependentDofs()-numLocalDofs;
    BodyNodeDynamics *nodeParentDyn = static_cast<BodyNodeDynamics*>(mParentNode);

    // Update Local Derivatives
    for(int i = 0; i < numLocalDofs; i++)
    {
        if(numParentDofs + i < mNumRootTrans)
            continue;   // since mTq is a constant

        for(int j=0; j<numLocalDofs; j++)
        {
            if(numParentDofs + j<mNumRootTrans)
                continue;   // since mTq is a constant

            mTqq.at(i).at(j) = getLocalSecondDeriv(getDof(i), getDof(j));
        }
    }

    // Update World Derivatives
    // parent dofs i
    for (int i = mNumRootTrans; i < numParentDofs; i++)
    {
        assert(nodeParentDyn);    // should always have a parent if enters this for loop
        // parent dofs j
        for(int j = mNumRootTrans; j < numParentDofs; j++)
        {
            mWqq.at(i).at(j) = nodeParentDyn->mWqq.at(i).at(j) * mT;
        }
        // local dofs j
        for(int j = 0; j < numLocalDofs; j++)
        {
            mWqq.at(i).at(numParentDofs + j) = nodeParentDyn->mWq.at(i) * mTq.at(j);
        }
    }
    // local dofs i
    for (int i = 0; i < numLocalDofs; i++)
    {
        if(numParentDofs + i<mNumRootTrans) continue;
        // parent dofs j
        for (int j = mNumRootTrans; j < numParentDofs; j++)
        {
            mWqq.at(numParentDofs + i).at(j) = nodeParentDyn->mWq.at(j) * mTq.at(i);
        }
        // local dofs j
        for (int j = 0; j < numLocalDofs; j++)
        {
            if (numParentDofs + j<mNumRootTrans)
                continue;

            if (nodeParentDyn)
                mWqq.at(numParentDofs + i).at(numParentDofs + j) = nodeParentDyn->mW * mTqq.at(i).at(j);
            else
                mWqq.at(numParentDofs + i).at(numParentDofs + j) = mTqq.at(i).at(j);
        }
    }

    for (int i = 0; i < numParentDofs+numLocalDofs; i++)
    {
        evalJacDerivLin(i, _offset);
        evalJacDerivAng(i);
    }
}

void BodyNodeDynamics::evalJacDerivLin(int _qi, Vector3d _offset)
{
    mJvq.at(_qi).setZero();

    if (_qi<mNumRootTrans)
        return;

    for (int j = mNumRootTrans; j < getNumDependentDofs(); j++)
    {
        VectorXd Jvqi = dart_math::xformHom(mWqq.at(_qi).at(j), _offset);
        mJvq.at(_qi)(0,j) = Jvqi[0];
        mJvq.at(_qi)(1,j) = Jvqi[1];
        mJvq.at(_qi)(2,j) = Jvqi[2];
    }
}

void BodyNodeDynamics::evalJacDerivAng(int _qi)
{
    mJwq.at(_qi).setZero();

    if (_qi < mNumRootTrans)
        return;

    for (int j = mNumRootTrans; j < getNumDependentDofs(); j++)
    {
        Matrix3d JwqijSkewSymm
                = mWqq.at(_qi).at(j).topLeftCorner<3,3>()*mW.topLeftCorner<3,3>().transpose()
                + mWq.at(j).topLeftCorner<3,3>()*mWq.at(_qi).topLeftCorner<3,3>().transpose();
        Vector3d Jwqij = dart_math::fromSkewSymmetric(JwqijSkewSymm);
        mJwq.at(_qi)(0,j) = Jwqij[0];
        mJwq.at(_qi)(1,j) = Jwqij[1];
        mJwq.at(_qi)(2,j) = Jwqij[2];
    }
}

void BodyNodeDynamics::evalJacDotLin(const VectorXd &_qDotSkel)
{
    mJvDot.setZero();

    for(int i = mNumRootTrans; i < getNumDependentDofs(); i++)
        mJvDot += mJvq.at(i)*_qDotSkel[mDependentDofs[i]];
}

void BodyNodeDynamics::evalJacDotAng(const VectorXd &_qDotSkel)
{
    mJwDot.setZero();

    for(int i = mNumRootTrans; i < getNumDependentDofs(); i++)
        mJwDot += mJwq.at(i)*_qDotSkel[mDependentDofs[i]];
}

void BodyNodeDynamics::evalMassMatrix()
{
    mM.triangularView<Upper>() = getMass() * mJv.transpose() * mJv;
    mM.triangularView<Upper>() += mJw.transpose() * mIc * mJw;
    mM.triangularView<StrictlyLower>() = mM.transpose();
}

void BodyNodeDynamics::evalCoriolisMatrix(const VectorXd &_qDotSkel)
{
    // evaluate the Dot terms
    evalJacDotLin(_qDotSkel);   // evaluates mJvDot
    evalJacDotAng(_qDotSkel);   // evaluates mJwDot
    evalOmega(_qDotSkel);   // evaluates mOmega vector

    // term 1
    mC.noalias() = getMass() * mJv.transpose() * mJvDot;
    mC.noalias() += mJw.transpose() * mIc * mJwDot;
    // term 2
    mC.noalias() += mJw.transpose() * dart_math::makeSkewSymmetric(mOmega) * mIc * mJw;
}

void BodyNodeDynamics::evalCoriolisVector(const VectorXd &_qDotSkel)
{
    // evaluate the Dot terms
    evalJacDotLin(_qDotSkel);   // evaluates mJvDot
    evalJacDotAng(_qDotSkel);   // evaluates mJwDot
    evalOmega(_qDotSkel);   // evaluates mOmega vector

    // term 1
    Vector3d Jvdqd = Vector3d::Zero();
    Vector3d Jwdqd = Vector3d::Zero();
    for(int i=mNumRootTrans; i<getNumDependentDofs(); i++)
    {
        Jvdqd += mJvDot.col(i)*_qDotSkel[mDependentDofs[i]];
        Jwdqd += mJwDot.col(i)*_qDotSkel[mDependentDofs[i]];
    }
    mCvec.noalias() = getMass() * mJv.transpose() * Jvdqd;
    mCvec.noalias() += mJw.transpose() * (mIc * Jwdqd);
    // term 2
    mCvec.noalias() += mJw.transpose() * (mOmega.cross(mIc*mOmega));

    //// test
    //evalCoriolisMatrix(_qDotSkel);
    //for(int i=0; i<mC.cols(); i++) mCvec += mC.col(i)*_qDotSkel[mDependentDofs[i]];
}

void BodyNodeDynamics::evalGravityVector(const Vector3d& _gravity)
{
    assert(mG.rows() == getNumDependentDofs() && mJv.cols() == getNumDependentDofs());

    for(unsigned int i = 0; i < getNumDependentDofs(); i++)
        mG[i] = -getMass()*_gravity.dot(mJv.col(i));    // '-' sign as term is on the left side of dynamics equation
}

void BodyNodeDynamics::evalExternalForces(VectorXd& _extForce)
{
    mFext = VectorXd::Zero(getNumDependentDofs());

    // contribution of linear force
    for(unsigned int i = 0; i < mContacts.size(); i++)
    {
        // compute J
        MatrixXd J = MatrixXd::Zero(3, getNumDependentDofs());
        Vector3d force = mW.topLeftCorner<3,3>() * mContacts[i].second;
        for(int j = 0; j < getNumDependentDofs(); j++)
            J.col(j) = dart_math::xformHom(mWq[j],mContacts[i].first);
        // compute J^TF
        mFext.noalias() += J.transpose() * force;
    }

    // contribution of torque
    if(mExtTorqueBody.norm() > 0)
        mFext.noalias() += mJw.transpose() * mW.topLeftCorner<3,3>() * mExtTorqueBody;

    for(int i = 0; i < getNumDependentDofs(); i++)
        _extForce(mDependentDofs[i]) += mFext(i);
}

void BodyNodeDynamics::evalExternalForcesRecursive(VectorXd& _extForce)
{
    for (unsigned int i = 0; i < mContacts.size(); i++)
    { // transform forces from com to joint
        mExtForceBody += mContacts[i].second;
        mExtTorqueBody += mContacts[i].first.cross(mContacts[i].second);
    }

    for (unsigned int i = 0; i < mJointsChild.size(); i++)
    { // recursion
        BodyNodeDynamics* childNode
                = (BodyNodeDynamics*)mJointsChild[i]->getChildNode();

        Matrix3d Rchild = childNode->mT.topLeftCorner<3,3>(); // rotation from parent to child
        Vector3d forceChild = Rchild*childNode->mExtForceBody; // convert external force of child to parent frame
        mExtForceBody += forceChild;

        Vector3d rlchild = childNode->mT.topRightCorner<3,1>(); // child com in parent frame
        mExtTorqueBody += rlchild.cross(forceChild);
        mExtTorqueBody.noalias() += Rchild * childNode->mExtTorqueBody; // torque induced by linear force in child node, and the torque in child node
    }

    // convert mExtForceBody and mExtTorqueBody from cartesian space to generalized coordinates (_extForce)
    jointCartesianToGeneralized(mExtTorqueBody, _extForce);
    if(getParentJoint()->getNumDofsTrans() > 0)
    {
        jointCartesianToGeneralized(mExtForceBody, _extForce, false);
    }
}

void BodyNodeDynamics::jointCartesianToGeneralized(const Vector3d& _cForce,
                                                   VectorXd& _gForce,
                                                   bool _isTorque )
{
    Joint* joint = getParentJoint();
    if (joint->getJointType() == Joint::J_UNKNOWN ||
        joint->getJointType() == Joint::J_TRANS)
        return;

    Matrix3d Ri = getLocalTransform().topLeftCorner<3,3>();
    if(_isTorque)
    {
        VectorXd torque = mJwJoint.transpose() * (Ri * _cForce);
        int firstRotDof = joint->getFirstRotDofIndex();
        for(int i=0; i<joint->getNumDofsRot(); i++)
            _gForce(firstRotDof+i) += torque(i);
    }
    else
    {
        if(joint->getNumDofsTrans()>0)
        {
            assert(joint->getNumDofsTrans()==3); // assume translational dofs are always for all three
            _gForce.segment<3>(joint->getFirstTransDofIndex()).noalias() += Ri * _cForce;
        }
    }
}

void BodyNodeDynamics::bodyCartesianToGeneralized(const Vector3d& _cForce,
                                                  VectorXd& _gForce,
                                                  bool _isTorque)
{
    Joint* joint = getParentJoint();

    if (joint->getJointType() == Joint::J_UNKNOWN )
        return;

    if( _isTorque)
    {
        jointCartesianToGeneralized( _cForce, _gForce, true );
    }
    else
    {
        Vector3d torque = mCOMLocal.cross(_cForce);
        jointCartesianToGeneralized( torque, _gForce, true );
        if (joint->getNumDofsTrans() > 0)
            jointCartesianToGeneralized(_cForce, _gForce, false);
    }
}

void BodyNodeDynamics::getGeneralized(VectorXd& _gForce)
{
    jointCartesianToGeneralized(mTorqueJointBody, _gForce);

    if(getParentJoint()->getNumDofsTrans() > 0)
        jointCartesianToGeneralized(mForceJointBody, _gForce, false);
}

void BodyNodeDynamics::aggregateMass(Eigen::MatrixXd &_M)
{
    for(int i=0; i<getNumDependentDofs(); i++)
        for(int j=0; j<getNumDependentDofs(); j++)
            _M(mDependentDofs[i], mDependentDofs[j]) += mM(i, j);
}
void BodyNodeDynamics::aggregateCoriolis(Eigen::MatrixXd &_C)
{
    for(int i=0; i<getNumDependentDofs(); i++)
        for(int j=0; j<getNumDependentDofs(); j++)
            _C(mDependentDofs[i], mDependentDofs[j]) += mC(i, j);
}

void BodyNodeDynamics::aggregateCoriolisVec(Eigen::VectorXd &_Cvec)
{
    for(int i=0; i<getNumDependentDofs(); i++)
        _Cvec[mDependentDofs[i]] += mCvec[i];
}

void BodyNodeDynamics::aggregateGravity(Eigen::VectorXd &_G)
{
    for (int i = 0; i<getNumDependentDofs(); i++)
        _G[mDependentDofs[i]] += mG[i];
}

void BodyNodeDynamics::addExtForce(const Vector3d& _offset,
                                   const Vector3d& _force,
                                   bool _isOffsetLocal,
                                   bool _isForceLocal)
{
    Vector3d pos = _offset;
    Vector3d force = _force;

    if (!_isOffsetLocal)
        pos = dart_math::xformHom(getWorldInvTransform(), _offset);

    if (!_isForceLocal)
        force.noalias() = mW.topLeftCorner<3,3>().transpose() * _force;

    mContacts.push_back(pair<Vector3d, Vector3d>(pos, force));
}

void BodyNodeDynamics::addExtTorque(const Vector3d& _torque, bool _isLocal)
{
    if (_isLocal)
        mExtTorqueBody += _torque;
    else
        mExtTorqueBody += mW.topLeftCorner<3,3>().transpose()*_torque;
}

void BodyNodeDynamics::clearExternalForces()
{
    mContacts.clear();
    mFext.setZero();
    mExtForceBody.setZero();
    mExtTorqueBody.setZero();
}

Vector3d BodyNodeDynamics::evalLinMomentum()
{
    return mMass * mVel;
}

Vector3d BodyNodeDynamics::evalAngMomentum(Vector3d _pivot)
{
    Vector3d d = getWorldCOM() - _pivot;
    Matrix3d Inew = mIc
                    + mMass * (d.dot(d) * Matrix3d::Identity() - d * d.transpose());
    //        evalOmega();
    //        cout << mOmega << endl;
    return Inew * mOmega;
}

}   // namespace dynamics
