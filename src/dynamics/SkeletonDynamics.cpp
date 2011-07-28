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

    VectorXd SkeletonDynamics::computeInverseDynamicsLinear( const Vector3d &_gravity, const VectorXd *_qdot, const VectorXd *_qdotdot, bool _computeJacobians ) {
        // FORWARD PASS: compute the velocities recursively - from root to end effectors
        for(int i=0; i<getNumNodes(); i++){ // increasing order ensures that the parent joints/nodes are evaluated before the child
            BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
            // init the node in the first pass
            nodei->initInverseDynamics();
            // compute the velocities
            nodei->computeInvDynVelocities(_gravity, _qdot, _qdotdot, _computeJacobians);
        }

        // BACKWARD PASS: compute the forces recursively -  from end effectors to root
        for(int i=getNumNodes()-1; i>=0; i--){ // decreasing order ensures that the parent joints/nodes are evaluated after the child
            BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
            nodei->computeInvDynForces(_gravity, _qdot, _qdotdot);
        }

        // convert all the joint torques to generalized coordinates and collect them into a vector
        VectorXd torqueGen = VectorXd::Zero(getNumDofs());
        assert(getNumNodes()==getNumJoints());
        for(int i=0; i<getNumNodes(); i++){ // increasing order ensures that the parent joints/nodes are evaluated before the child
            BodyNodeDynamics *bnodei = static_cast<BodyNodeDynamics*>(mNodes[i]);
            // convert to generalized torques
            Matrix3d Ri = bnodei->getLocalTransform().topLeftCorner(3,3);
            VectorXd torqueGeni = bnodei->mJwJoint.transpose()*Ri*bnodei->mTorqueJointBody;
            // translation dof forces may add to torque as well: e.g. Root joint
            if(mJoints[i]->getNumDofsTrans()>0){
                assert(mJoints[i]->getNumDofsTrans()==3);   // ASSUME: 3 DOF translation only
                torqueGen.segment(mJoints[i]->getFirstTransDofIndex(), mJoints[i]->getNumDofsTrans()) = Ri*bnodei->mForceJointBody;
            }
            torqueGen.segment(mJoints[i]->getFirstRotDofIndex(), mJoints[i]->getNumDofsRot()) = torqueGeni;
        }

        return torqueGen;
    }

    void SkeletonDynamics::computeDynamics(const Vector3d &_gravity, const VectorXd &_qdot, bool _useInvDynamics){
        mM = MatrixXd::Zero(getNumDofs(), getNumDofs());
        //mC = MatrixXd::Zero(getNumDofs(), getNumDofs());
        mCvec = VectorXd::Zero(getNumDofs());
        mG = VectorXd::Zero(getNumDofs());
        mCg = VectorXd::Zero(getNumDofs());
        if(_useInvDynamics){
            mCg = computeInverseDynamicsLinear(_gravity, &_qdot, NULL, true);
            for(int i=0; i<getNumNodes(); i++){
                BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
                // mass matrix M
                nodei->evalMassMatrix(); // assumes Jacobians mJv and mJw have been computed above: use flag as true in computeInverseDynamicsLinear
                nodei->addMass(mM);
            }
        }
        else {
            // init the data structures for the dynamics
            for(int i=0; i<getNumNodes(); i++){
                BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(getNode(i));
                // initialize the data structures for storage
                nodei->initDynamics();

                // compute the transforms and the derivatives
                nodei->updateTransform();
                nodei->updateFirstDerivatives();
                nodei->updateSecondDerivatives();
                // compute the required data structures and add to the skel's data structures
                // mass matrix M
                nodei->evalMassMatrix();
                nodei->addMass(mM);
                //// Coriolis C
                //nodei->evalCoriolisMatrix(_qdot);
                //nodei->addCoriolis(mC);
                // Coriolis vector C*qd
                nodei->evalCoriolisVector(_qdot);
                nodei->addCoriolisVec(mCvec);
                // gravity vector G
                nodei->evalGravityVector(_gravity);
                nodei->addGravity(mG);
            }

            //mCg = mC*_qdot + mG;
            mCg = mCvec + mG;
        }
    }

}   // namespace dynamics
