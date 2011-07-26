/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author  Sumit Jain
    Date    07/21/2011
*/

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "model3d/Joint.h"
#include "model3d/BodyNode.h"

using namespace Eigen;
using namespace model3d;

namespace dynamics{
    SkeletonDynamics::SkeletonDynamics(): model3d::Skeleton(){
    }

    SkeletonDynamics::~SkeletonDynamics(){
    }

    model3d::BodyNode* SkeletonDynamics::createBodyNode(const char* const _name){
        return new BodyNodeDynamics(_name);
    }

    VectorXd SkeletonDynamics::inverseDynamicsLinear( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot ) {
        // FORWARD PASS: compute the velocities recursively - from root to end effectors
        for(int i=0; i<mNumNodes; i++){ // increasing order ensures that the parent joints/nodes are evaluated before the child
            BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
            nodei->computeInvDynVelocities(_gravity, _qdot, _qdotdot);
        }

        // BACKWARD PASS: compute the forces recursively -  from end effectors to root
        for(int i=mNumNodes-1; i>=0; i--){ // decreasing order ensures that the parent joints/nodes are evaluated after the child
            BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
            nodei->computeInvDynForces(_gravity, _qdot, _qdotdot);
        }

        // convert all the joint torques to generalized coordinates and collect them into a vector
        VectorXd torqueGen = VectorXd::Zero(getNumDofs());
        assert(mNumNodes==mNumJoints);
        for(int i=0; i<mNumNodes; i++){ // increasing order ensures that the parent joints/nodes are evaluated before the child
            BodyNodeDynamics *bnodei = static_cast<BodyNodeDynamics*>(mNodes[i]);
            // convert to generalized torques
            Matrix3d Ri = bnodei->getLocalTransform().topLeftCorner(3,3);
            VectorXd torqueGeni = bnodei->mJwJoint.transpose()*Ri*bnodei->mTorqueJointLocal;
            // translation dof forces may add to torque as well: e.g. Root joint
            if(mJoints[i]->getNumDofsTrans()>0){
                assert(mJoints[i]->getNumDofsTrans()==3);   // ASSUME: 3 DOF translation only
                Vector3d cl = bnodei->getLocalCOM();
                torqueGen.segment(mJoints[i]->getFirstTransDofIndex(), bnodei->getParentJoint()->getNumDofsTrans()) = Ri*bnodei->mForceJointLocal;
                torqueGeni += bnodei->mJwJoint.transpose()*Ri*(-cl.cross(bnodei->mForceJointLocal));
            }
            torqueGen.segment(mJoints[i]->getFirstRotDofIndex(), bnodei->getParentJoint()->getNumDofsRot()) = torqueGeni;
        }

        return torqueGen;
    }

}   // namespace dynamics
