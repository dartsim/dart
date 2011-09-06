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
#include "model3d/Dof.h"
#include "utils/UtilsMath.h"

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
            if(bnodei->getParentJoint()->getJointType()==Joint::J_UNKNOWN) continue;

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

    // assumptions made:
    // 1. the pose _q has already been set to the model
    // 2. first derivatives are up-to-date (i.e. consistent with _q ) 
    // XXX
    ///@Warning: dof values and transformations are inconsistent after this computation!
    void SkeletonDynamics::clampRotation(VectorXd& _q, VectorXd& _qdot){
        for(unsigned int i=0; i<mJoints.size(); i++){
            Joint* jnt = mJoints.at(i);
            switch(jnt->getJointType()){ // only cares about euler and expmap of 3 rotations 
            case Joint::J_FREEEULER:
            case Joint::J_BALLEULER:
            {
                // clamp dof values to the range
                for(int j=0; j<3; j++){
                    int dofIndex = jnt->getDof(j)->getSkelIndex();
                    if( _q[dofIndex]>M_PI ) _q[dofIndex] -= 2*M_PI;
                    else if( _q[dofIndex]<-M_PI ) _q[dofIndex] += 2*M_PI;
                }
            }   break;
            case Joint::J_FREEEXPMAP:
            case Joint::J_BALLEXPMAP:
            {
                Vector3d exmap;
                for(int j=0; j<3; j++){
                    int dofIndex = jnt->getDof(j)->getSkelIndex();
                    exmap[j] = _q[dofIndex];
                }
                
                double theta = exmap.norm();
                if( theta > M_PI ){
                    exmap.normalize();
                    exmap *= theta-2*M_PI;
                
                    BodyNode* node = jnt->getChildNode();
                    int firstIndex = 0;
                    if( jnt->getParentNode()!=NULL )
                        firstIndex = jnt->getParentNode()->getNumDependentDofs();
                        
                    // extract the local Jw
                    Matrix3d oldJwBody;
                    for(int j=0; j<3; j++){
                        Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j)).topLeftCorner(3,3)
                            * node->getLocalTransform().topLeftCorner(3,3).transpose();
                        oldJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                    }
                
                    // the new Jw
                    Matrix3d newJwBody;
                    // set new dof values to joint for derivative evaluation
                    for(int j=0; j<3; j++)
                        jnt->getDof(j)->setValue(exmap[j]);

                    // compute the new Jw from Rq*trans(R)
                    Matrix4d Tbody = jnt->getLocalTransform();
                    for(int j=0; j<3; j++){
                        Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j)).topLeftCorner(3,3)*Tbody.topLeftCorner(3,3).transpose();
                        newJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                    }

                    // solve new_qdot s.t. newJw*new_qdot = w
                    // new_qdot = newJw.inverse()*w
                    Vector3d old_qdot; // extract old_qdot
                    for(int j=0; j<3; j++)
                        old_qdot[j] = _qdot[jnt->getDof(j)->getSkelIndex()];
                    Vector3d new_qdot = newJwBody.inverse()*oldJwBody*old_qdot;
                    for(int j=0; j<3; j++){
                        int dofIndex = jnt->getDof(j)->getSkelIndex();
                        _q[dofIndex] = exmap[j];
                        _qdot[dofIndex] = new_qdot[j];
                    }
                }
            }   break;
            default:
                break;
            }
        }
    }

}   // namespace dynamics
