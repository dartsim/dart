#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/ClosedLoopConstraint.h"
#include "kinematics/Dof.h"
#include "kinematics/Shape.h"
#include "math/UtilsMath.h"
#include "collision/CollisionDetector.h"

using namespace kinematics;
using namespace dynamics;
using namespace dart_math;

Controller::Controller(SkeletonDynamics* _skel, ConstraintDynamics* _constr, double _t) {
    mSkel = _skel;
    mConstraintHandle = _constr;
    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques = VectorXd::Zero(nDof);
    mDesiredDofs.resize(nDof);
    
    mDesiredDofs = mSkel->getPose();
    
    // Using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body + lower back
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 100.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 10.0;

}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    /*
    // SPD tracking
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstraintHandle->getTotalConstraintForce(0));
    mTorques = p + d - mKd * qddot * mTimestep;
    */

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}

void Controller::leftHandGrab() {
    int nContact = mConstraintHandle->getCollisionChecker()->getNumContacts();

    for (int i = 0; i < nContact; i++) {
        Vector3d pt = mConstraintHandle->getCollisionChecker()->getContact(i).point;
        BodyNodeDynamics *bd1 = (BodyNodeDynamics*)mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode();
        BodyNodeDynamics *bd2 = (BodyNodeDynamics*)mConstraintHandle->getCollisionChecker()->getContact(i).collisionNode2->getBodyNode();
        if(bd1 == (BodyNodeDynamics*)mSkel->getNode("fullbody2_h_hand_left")) {
            bd1->setCollideState(false);
            Vector3d offset1 = xformHom(bd1->getWorldInvTransform(), pt);
            Vector3d offset2 = xformHom(bd2->getWorldInvTransform(), pt);
            ClosedLoopConstraint *constr = new ClosedLoopConstraint(bd1, bd2, offset1, offset2, 0, 1);
            mConstraintHandle->addConstraint(constr);
            break;
        } else if (bd2 == (BodyNodeDynamics*)mSkel->getNode("fullbody2_h_hand_left")) {
            bd2->setCollideState(false);
            Vector3d offset1 = xformHom(bd1->getWorldInvTransform(), pt);
            Vector3d offset2 = xformHom(bd2->getWorldInvTransform(), pt);
            ClosedLoopConstraint *constr = new ClosedLoopConstraint(bd1, bd2, offset1, offset2, 1, 0);
            mConstraintHandle->addConstraint(constr);
            break;
        } 
    }
}

void Controller::leftHandRelease() {
    int nConstr = mConstraintHandle->getNumConstraints();
    
    for (int i = nConstr - 1; i >= 0; i--) {
        Constraint* c = mConstraintHandle->getConstraint(i);
        if (ClosedLoopConstraint* closedConstr = dynamic_cast<ClosedLoopConstraint*>(c))
            if (closedConstr->getBody1() == mSkel->getNode("fullbody2_h_hand_left") || closedConstr->getBody2() == mSkel->getNode("fullbody2_h_hand_left"))
                mConstraintHandle->deleteConstraint(closedConstr);
    }
}
