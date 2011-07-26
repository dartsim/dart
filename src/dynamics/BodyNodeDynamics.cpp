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
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace model3d;

namespace dynamics{
    BodyNodeDynamics::BodyNodeDynamics( const char *_name ) : BodyNode(_name){
        mJwJoint = MatrixXd::Zero(3,0);
        mJwDotJoint = MatrixXd::Zero(3,0);
        mVLocal = Vector3d::Zero();
        mVDotLocal = Vector3d::Zero();
        mOmegaLocal = Vector3d::Zero();
        mOmegaDotLocal = Vector3d::Zero();
    }

    BodyNodeDynamics::~BodyNodeDynamics(){
    }

    void BodyNodeDynamics::computeInvDynVelocities( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot ) {
        // update the local transform mT and the world transform mW
        BodyNode::updateTransform();

        mVLocal.setZero();
        mVDotLocal.setZero();
        mOmegaLocal.setZero();
        mOmegaDotLocal.setZero();

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

            mOmegaLocal = RjointT*(nodeParent->mOmegaLocal + omegaJoint);
            mOmegaDotLocal = RjointT*(nodeParent->mOmegaDotLocal + omegaDotJoint + nodeParent->mOmegaDotLocal.cross(omegaDotJoint));
            mVLocal = mOmegaLocal.cross(cl) + RjointT*(nodeParent->mVLocal + nodeParent->mOmegaLocal.cross(rl-clparent));
            mVDotLocal = mOmegaDotLocal.cross(cl) + mOmegaLocal.cross(mOmegaLocal.cross(cl)) + RjointT*(nodeParent->mVDotLocal + nodeParent->mOmegaDotLocal.cross(rl-clparent) + nodeParent->mOmegaLocal.cross(nodeParent->mOmegaLocal.cross(rl-clparent)));
        }
        // base case: root
        else {
            mOmegaLocal = RjointT*omegaJoint;
            mOmegaDotLocal = RjointT*omegaDotJoint;
            mVLocal = mOmegaLocal.cross(cl);
            mVDotLocal = mOmegaDotLocal.cross(cl) + mOmegaLocal.cross(mOmegaLocal.cross(cl));
            // incorporate gravity as part of acceleration by changing frame to the one accelerating with g
            // Therefore real acceleration == W*mVDotLocal + g;
            mVDotLocal -= RjointT*_gravity;
            // if root has translation DOFs
            if(mJointParent->getNumDofsTrans()>0){
                VectorXd qDotTransJoint = _qdot->segment(mJointParent->getFirstTransDofIndex(), mJointParent->getNumDofsTrans());
                mVLocal += RjointT*qDotTransJoint;
                if(_qdotdot){
                    VectorXd qDotDotTransJoint = _qdotdot->segment(mJointParent->getFirstTransDofIndex(), mJointParent->getNumDofsTrans());
                    mVDotLocal += RjointT*qDotDotTransJoint;
                }
            }
        }
    }

    void BodyNodeDynamics::computeInvDynForces( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot ) {
        cout<<"BodyNodeDynamics::computeInvDynForces - Not yet implemented\n";
        mForceJointLocal.setZero();
        mTorqueJointLocal.setZero();

        Vector3d cl = mCOMLocal;
        Matrix3d Ic = mPrimitive->getInertia();

        // base case: end effectors
        mForceJointLocal = mMass*mVDotLocal;
        mTorqueJointLocal = cl.cross(mForceJointLocal) + mOmegaLocal.cross(Ic*mOmegaLocal) + Ic*mOmegaDotLocal;
        // general case
        for(unsigned int j=0; j<mJointsChild.size(); j++){
            BodyNodeDynamics *bchild = static_cast<BodyNodeDynamics*>(mJointsChild[j]->getChildNode());
            Matrix3d Rchild = bchild->mT.topLeftCorner(3,3);
            Vector3d forceChildNode = Rchild*bchild->mForceJointLocal;
            mForceJointLocal += forceChildNode;
            Vector3d rlchild = bchild->mT.col(3).head(3);
            mTorqueJointLocal += (rlchild-cl).cross(forceChildNode) + Rchild*bchild->mTorqueJointLocal;
        }

        //// convert to generalized torques
        //mTorqueGen.segment(mJointParent->getFirstRotDofIndex(), mJointParent->getNumDofsRot()) = mJwJoint.transpose()*mT.topLeftCorner(3,3)*mTorqueJointLocal;
    }



}   // namespace dynamics
