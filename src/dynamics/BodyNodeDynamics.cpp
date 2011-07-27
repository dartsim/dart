/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author  Sumit Jain
    Date    07/21/2011
*/

#include "BodyNodeDynamics.h"
#include "model3d/Joint.h"
#include "model3d/Primitive.h"
#include "model3d/Transformation.h"
#include "utils/UtilsMath.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace model3d;

namespace dynamics{
    BodyNodeDynamics::BodyNodeDynamics( const char *_name ) : BodyNode(_name){
        mJwJoint = MatrixXd::Zero(3,0);
        mJwDotJoint = MatrixXd::Zero(3,0);
        mVelBody = Vector3d::Zero();
        mVelDotBody = Vector3d::Zero();
        mOmegaBody = Vector3d::Zero();
        mOmegaDotBody = Vector3d::Zero();
    }

    BodyNodeDynamics::~BodyNodeDynamics(){
    }

    void BodyNodeDynamics::initInverseDynamics(){
        BodyNode::init();

        mJwJoint = MatrixXd::Zero(3, getNumDependantDofs());
        mJwDotJoint = MatrixXd::Zero(3, getNumDependantDofs());
        mVelBody.setZero();
        mOmegaBody.setZero();
        mVelDotBody.setZero();
        mOmegaDotBody.setZero();
        mForceJointBody.setZero();
        mTorqueJointBody.setZero();
        
        // mJv, mJw, and mIc needed for the mass matrix are already inited and computed by the model3d::BodyNode
    }

    void BodyNodeDynamics::initDynamics(){
        BodyNode::init();
         
        mVel.setZero();
        mOmega.setZero();
        mM = MatrixXd::Zero(getNumDependantDofs(), getNumDependantDofs());
        mC = MatrixXd::Zero(getNumDependantDofs(), getNumDependantDofs());
        mCvec = VectorXd::Zero(getNumDependantDofs());
        mG = VectorXd::Zero(getNumDependantDofs());
        mTqq.resize(getNumLocalDofs());
        for(int i=0; i<getNumLocalDofs(); i++) mTqq[i].resize(getNumLocalDofs(), Matrix4d::Zero());
        mWqq.resize(getNumDependantDofs());
        for(int i=0; i<getNumDependantDofs(); i++) mWqq[i].resize(getNumDependantDofs(), Matrix4d::Zero());
        mJvq.resize(getNumDependantDofs(), MatrixXd::Zero(3, getNumDependantDofs()));
        mJwq.resize(getNumDependantDofs(), MatrixXd::Zero(3, getNumDependantDofs()));
        mJvDot = MatrixXd::Zero(3, getNumDependantDofs());
        mJwDot = MatrixXd::Zero(3, getNumDependantDofs());
    }

    void BodyNodeDynamics::computeInvDynVelocities( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot ) {
        // update the local transform mT and the world transform mW
        BodyNode::updateTransform();

        mVelBody.setZero();
        mVelDotBody.setZero();
        mOmegaBody.setZero();
        mOmegaDotBody.setZero();

        mJwJoint = MatrixXd::Zero(3, mJointParent->getNumDofsRot());
        mJwDotJoint = MatrixXd::Zero(3, mJointParent->getNumDofsRot());
         // ASSUME: trans dofs before rotation dofs
        VectorXd qDotJoint = _qdot->segment(mJointParent->getFirstRotDofIndex(), mJointParent->getNumDofsRot());
        mJointParent->computeRotationJac(&mJwJoint, &mJwDotJoint, &qDotJoint);

        // Local Rotation matrix transposed
        Matrix3d RjointT = mT.topLeftCorner(3,3).transpose();

        // Local angular velocity and angular acceleration
        Vector3d omegaJoint = mJwJoint*qDotJoint;
        Vector3d omegaDotJoint = mJwDotJoint*qDotJoint;
        if(_qdotdot){
            VectorXd qDotDotJoint = _qdotdot->segment(mJointParent->getFirstRotDofIndex(), mJointParent->getNumDofsRot());
            omegaDotJoint += mJwJoint*qDotDotJoint;
        }

        BodyNodeDynamics *nodeParent = static_cast<BodyNodeDynamics*>(mNodeParent);
        Vector3d cl = mCOMLocal;
        if(nodeParent){
            assert(mJointParent->getNumDofsTrans()==0); // assuming no internal translation dofs
            Vector3d clparent = nodeParent->getLocalCOM();
            Vector3d rl = mT.col(3).head(3);    // translation from parent's origin to self origin

            mOmegaBody = RjointT*(nodeParent->mOmegaBody + omegaJoint);
            mOmegaDotBody = RjointT*(nodeParent->mOmegaDotBody + omegaDotJoint + nodeParent->mOmegaBody.cross(omegaJoint));
            mVelBody = mOmegaBody.cross(cl) + RjointT*(nodeParent->mVelBody + nodeParent->mOmegaBody.cross(rl-clparent));
            mVelDotBody = mOmegaDotBody.cross(cl) + mOmegaBody.cross(mOmegaBody.cross(cl)) + RjointT*(nodeParent->mVelDotBody + nodeParent->mOmegaDotBody.cross(rl-clparent) + nodeParent->mOmegaBody.cross(nodeParent->mOmegaBody.cross(rl-clparent)));
        }
        // base case: root
        else {
            mOmegaBody = RjointT*omegaJoint;
            mOmegaDotBody = RjointT*omegaDotJoint;
            mVelBody = mOmegaBody.cross(cl);
            mVelDotBody = mOmegaDotBody.cross(cl) + mOmegaBody.cross(mOmegaBody.cross(cl));
            // incorporate gravity as part of acceleration by changing frame to the one accelerating with g
            // Therefore real acceleration == W*mVelDotBody + g;
            mVelDotBody -= RjointT*_gravity;
            // if root has translation DOFs
            if(mJointParent->getNumDofsTrans()>0){
                VectorXd qDotTransJoint = _qdot->segment(mJointParent->getFirstTransDofIndex(), mJointParent->getNumDofsTrans());
                mVelBody += RjointT*qDotTransJoint;
                if(_qdotdot){
                    VectorXd qDotDotTransJoint = _qdotdot->segment(mJointParent->getFirstTransDofIndex(), mJointParent->getNumDofsTrans());
                    mVelDotBody += RjointT*qDotDotTransJoint;
                }
            }
        }
    }

    void BodyNodeDynamics::computeInvDynForces( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot ) {
        mForceJointBody.setZero();
        mTorqueJointBody.setZero();

        Vector3d cl = mCOMLocal;
        Matrix3d Ibody = mPrimitive->getInertia();

        // base case: end effectors
        mForceJointBody = mMass*mVelDotBody;
        mTorqueJointBody = cl.cross(mMass*mVelDotBody) + mOmegaBody.cross(Ibody*mOmegaBody) + Ibody*mOmegaDotBody;
        // general case
        for(unsigned int j=0; j<mJointsChild.size(); j++){
            BodyNodeDynamics *bchild = static_cast<BodyNodeDynamics*>(mJointsChild[j]->getChildNode());
            Matrix3d Rchild = bchild->mT.topLeftCorner(3,3);
            Vector3d forceChildNode = Rchild*bchild->mForceJointBody;
            mForceJointBody += forceChildNode;
            Vector3d rlchild = bchild->mT.col(3).head(3);
            mTorqueJointBody += (rlchild).cross(forceChildNode) + Rchild*bchild->mTorqueJointBody;
        }
    }

    Matrix4d BodyNodeDynamics::getLocalSecondDeriv(const Dof *_q1,const Dof *_q2 ) const {
        return mJointParent->getSecondDeriv(_q1, _q2);
    }

    void BodyNodeDynamics::updateSecondDerivatives() {
        const int numLocalDofs = getNumLocalDofs();
        const int numParentDofs = getNumDependantDofs()-numLocalDofs;
        BodyNodeDynamics *nodeParentDyn = static_cast<BodyNodeDynamics*>(mNodeParent);

        // Update Local Derivatives
        for(int i = 0; i < numLocalDofs; i++) {
            for(int j=0; j<numLocalDofs; j++) {
                mTqq.at(i).at(j) = getLocalSecondDeriv(getDof(i), getDof(j));
            }
        }

        // Update World Derivatives
        // parent dofs i
        for (int i = 0; i < numParentDofs; i++) {
            assert(nodeParentDyn);    // should always have a parent if enters this for loop
            // parent dofs j
            for(int j = 0; j < numParentDofs; j++) {
                mWqq.at(i).at(j) = nodeParentDyn->mWqq.at(i).at(j) * mT;
            }
            // local dofs j
            for(int j = 0; j < numLocalDofs; j++) {
                mWqq.at(i).at(numParentDofs + j) = nodeParentDyn->mWq.at(i) * mTq.at(j);
            }
        }
        // local dofs i
        for(int i = 0; i < numLocalDofs; i++){
            // parent dofs j
            for(int j = 0; j < numParentDofs; j++) {
                mWqq.at(numParentDofs + i).at(j) = nodeParentDyn->mWq.at(j) * mTq.at(i);
            }
            // local dofs j
            for(int j = 0; j < numLocalDofs; j++) {
                if(nodeParentDyn) mWqq.at(numParentDofs + i).at(numParentDofs + j) = nodeParentDyn->mW * mTqq.at(i).at(j);
                else mWqq.at(numParentDofs + i).at(numParentDofs + j) = mTqq.at(i).at(j);
            }
        }

        for(int i=0; i<numParentDofs+numLocalDofs; i++){
            evalJacDerivLin(i);
            evalJacDerivAng(i);
        }
    }

    void BodyNodeDynamics::evalJacDerivLin(int _qi){
        mJvq.at(_qi).setZero();
        for(int j=0; j<getNumDependantDofs(); j++){
            VectorXd Jvqi = utils::xformHom(mWqq.at(_qi).at(j), getLocalCOM());
            mJvq.at(_qi)(0,j) = Jvqi[0];
            mJvq.at(_qi)(1,j) = Jvqi[1];
            mJvq.at(_qi)(2,j) = Jvqi[2];
        }
    }

    void BodyNodeDynamics::evalJacDerivAng(int _qi){
        mJwq.at(_qi).setZero();
        for(int j=0; j<getNumDependantDofs(); j++){
            Matrix3d JwqijSkewSymm = mWqq.at(_qi).at(j).topLeftCorner(3,3)*mW.topLeftCorner(3,3).transpose() + mWq.at(j).topLeftCorner(3,3)*mWq.at(_qi).topLeftCorner(3,3).transpose();
            Vector3d Jwqij = utils::fromSkewSymmetric(JwqijSkewSymm);
            mJwq.at(_qi)(0,j) = Jwqij[0];
            mJwq.at(_qi)(1,j) = Jwqij[1];
            mJwq.at(_qi)(2,j) = Jwqij[2];
        }
    }

    void BodyNodeDynamics::evalJacDotLin(const VectorXd &_qDotSkel) {
        mJvDot.setZero();
        for(int i=0; i<getNumDependantDofs(); i++){
            mJvDot += mJvq.at(i)*_qDotSkel[mDependantDofs[i]];
        }
    }

    void BodyNodeDynamics::evalJacDotAng(const VectorXd &_qDotSkel) {
        mJwDot.setZero();
        for(int i=0; i<getNumDependantDofs(); i++){
            mJwDot += mJwq.at(i)*_qDotSkel[mDependantDofs[i]];
        }
    }

    void BodyNodeDynamics::evalVelocity(const VectorXd &_qDotSkel){
        mVel.setZero();
        for(int i=0; i<getNumDependantDofs(); i++){
            mVel += mJv.col(i)*_qDotSkel[mDependantDofs[i]];
        }
    }

    void BodyNodeDynamics::evalOmega(const VectorXd &_qDotSkel){
        mOmega.setZero();
        for(int i=0; i<getNumDependantDofs(); i++){
            mOmega += mJw.col(i)*_qDotSkel[mDependantDofs[i]];
        }
    }

    void BodyNodeDynamics::evalMassMatrix(){
        mM.setZero();
        mM = getMass()*mJv.transpose()*mJv + mJw.transpose()*mIc*mJw;
    }

    void BodyNodeDynamics::evalCoriolisMatrix(const VectorXd &_qDotSkel){
        mC.setZero();
        // evaluate the Dot terms
        evalJacDotLin(_qDotSkel);   // evaluates mJvDot
        evalJacDotAng(_qDotSkel);   // evaluates mJwDot
        evalOmega(_qDotSkel);   // evaluates mOmega vector

        Matrix3d R = mW.topLeftCorner(3,3);
        // term 1
        mC = getMass()*mJv.transpose()*mJvDot + mJw.transpose()*mIc*mJwDot;
        // term 2
        mC += mJw.transpose()*utils::makeSkewSymmetric(mOmega)*mIc*mJw;
    }

    void BodyNodeDynamics::evalCoriolisVector(const VectorXd &_qDotSkel){
        mCvec.setZero();
        // evaluate the Dot terms
        evalJacDotLin(_qDotSkel);   // evaluates mJvDot
        evalJacDotAng(_qDotSkel);   // evaluates mJwDot
        evalOmega(_qDotSkel);   // evaluates mOmega vector

        Matrix3d R = mW.topLeftCorner(3,3);
        // term 1
        Vector3d Jvdqd = Vector3d::Zero();
        Vector3d Jwdqd = Vector3d::Zero();
        for(int i=0; i<getNumDependantDofs(); i++){
            Jvdqd += mJvDot.col(i)*_qDotSkel[mDependantDofs[i]];
            Jwdqd += mJwDot.col(i)*_qDotSkel[mDependantDofs[i]];
        }
        mCvec = getMass()*mJv.transpose()*Jvdqd + mJw.transpose()*(mIc*Jwdqd);
        // term 2
        mCvec += mJw.transpose()*(mOmega.cross(mIc*mOmega));

        //// test
        //evalCoriolisMatrix(_qDotSkel);
        //for(int i=0; i<mC.cols(); i++) mCvec += mC.col(i)*_qDotSkel[mDependantDofs[i]];
    }

    void BodyNodeDynamics::evalGravityVector(const Vector3d &_gravity){
        mG.setZero();
        for(int i=0; i<mJv.cols(); i++){
            mG[i] = -getMass()*_gravity.dot(mJv.col(i));    // '-' sign as term is on the left side of dynamics equation
        }
    }

    void BodyNodeDynamics::addMass(Eigen::MatrixXd &_M){
        for(int i=0; i<getNumDependantDofs(); i++){
            for(int j=0; j<getNumDependantDofs(); j++){
                _M(mDependantDofs[i], mDependantDofs[j]) += mM(i, j);
            }
        }
    }
    void BodyNodeDynamics::addCoriolis(Eigen::MatrixXd &_C){
        for(int i=0; i<getNumDependantDofs(); i++){
            for(int j=0; j<getNumDependantDofs(); j++){
                _C(mDependantDofs[i], mDependantDofs[j]) += mC(i, j);
            }
        }
    }
    void BodyNodeDynamics::addCoriolisVec(Eigen::VectorXd &_Cvec){
        for(int i=0; i<getNumDependantDofs(); i++){
            _Cvec[mDependantDofs[i]] += mCvec[i];
        }
    }
    void BodyNodeDynamics::addGravity(Eigen::VectorXd &_G){
        for(int i=0; i<getNumDependantDofs(); i++){
            _G[mDependantDofs[i]] += mG[i];
        }
    }


}   // namespace dynamics
