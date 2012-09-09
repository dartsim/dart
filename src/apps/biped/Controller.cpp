#include "Controller.h"
<<<<<<< HEAD
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
=======

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
>>>>>>> karen
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/Shape.h"
<<<<<<< HEAD
=======
#include "kinematics/Marker.h"
#include "utils/UtilsMath.h"
#include "optimizer/snopt/SnoptSolver.h"
#include "optimizer/ConstraintBox.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/Var.h"
#include "collision/CollisionSkeleton.h"

#include "LandingProblem.h"
#include "PositionConstraint.h"
#include "COMConstraint.h"
>>>>>>> karen

using namespace kinematics;
using namespace Eigen;
using namespace utils;
<<<<<<< HEAD

Controller::Controller(dynamics::SkeletonDynamics *_skel, double _t) {
    mSkel = _skel;
=======
using namespace optimizer;
using namespace dynamics;

Controller::Controller(dynamics::SkeletonDynamics *_skel, dynamics::ContactDynamics *_collisionHandle, double _t) {
    mSkel = _skel;
    mCollisionHandle = _collisionHandle;
>>>>>>> karen
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    mFrame = 0;
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
<<<<<<< HEAD
       
    for (int i = 6; i < nDof; i++) {
        mKp(i, i) = 25.0;
        mKd(i, i) = 2 * sqrt(25.0);
    }

    // hips
    mKp(6, 6) = 500.0;
    //    mKd(6, 6) = 0.5 * sqrt(mKp(6, 6));
    mKp(12, 12) = 500.0;
    //    mKd(12, 12) = 0.5 * sqrt(mKp(12, 12));
    
    // knees
    mKp(9, 9) = 500.0;
    //    mKd(9, 9) = 0.5 * sqrt(mKp(9, 9));
    mKp(15, 15) = 500.0;
    //    mKd(15, 15) = 0.5 * sqrt(mKd(15, 15));
    
    // ankles
    mKp(10, 10) = 500.0;
    //    mKd(10, 10) = 0.5 * sqrt(mKp(10, 10));
    mKp(11, 11) = 500.0;
    mKd(11, 11) = 2 * sqrt(mKp(11, 11));
    mKp(16, 16) = 500.0;
    mKd(16, 16) = 0.5 * sqrt(mKp(16, 16));
    mKp(17, 17) = 500.0;
    mKd(17, 17) = 2 * sqrt(mKp(17, 17));
    
    // lower back
        mKp(18, 18) = 500.0;
    //    mKd(18, 18) = 2 * sqrt(mKp(18, 18));
        mKp(19, 19) = 500.0;
    //    mKd(19, 19) = 2 * sqrt(mKp(19, 19));
=======

    // using SPD results in simple Kp coefficients
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0;
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 100.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 10.0;
>>>>>>> karen

    // mass tree is only used when SPD is not activated
    mMassTree = VectorXd::Zero(nDof);
    for (int i = 6; i < nDof; i++)
        mMassTree[i] = computeMassTree(mSkel->getDof(i)->getJoint()->getChildNode());
    
    mMassTree /= mMassTree.norm();
    
<<<<<<< HEAD
}

void Controller::computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel) {
    mFrame++;
=======
    mJumpState = GROUND;
    mPrepareFrame = 0;
    mLandingFrame = 0;
    // use optimization to solve for a landing pose
    mProblem = new LandingProblem(mSkel);
    mSolver = new snopt::SnoptSolver(mProblem);
    // for plotting momentum
    mOutFile.open("temp.data");
    mControlBias = VectorXd::Zero(nDof);
    mConstrForces = VectorXd::Zero(nDof);

}

VectorXd Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    detectJumpState();

    if (mJumpState == TAKEOFF) {
        computeLandingPose();
        mPrepareFrame = 0;
        mJumpState = AIRBORNE;
    }
    if (mJumpState == AIRBORNE) {
        trackLandingPose(_dof, _dofVel);
    }
    if (mJumpState == LANDING) {
        maintainBalance(_dof, _dofVel);
    }
    if (mJumpState == GROUND){
        prepareJump(_dof, _dofVel);
    }
    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++){        
        mTorques[i] = 0.0;
    }

    mFrame++;
    return mControlBias;
}

void Controller::prepareJump(const VectorXd& _dof, const VectorXd& _dofVel) {
>>>>>>> karen
    int nDof = mSkel->getNumDofs();
    mTorques.setZero();
    MatrixXd J(MatrixXd::Zero(3, nDof));
    
<<<<<<< HEAD
=======
    // virtual force on lower body
    Vector3d vf(0.0, 0.0, 0.0);
    BodyNode *lFoot = mSkel->getNode("fullbody1_h_toe_left");
    BodyNode *rFoot = mSkel->getNode("fullbody1_h_toe_right");
    double footLen = lFoot->getShape()->getDim().maxCoeff();
    if (mPrepareFrame > 50) {
        vf[1] = -1000; // virtual force that pushes ground vertically from the center of the foot
        //footLen /= 2;
    }
    if (mPrepareFrame > 350) {
        vf[1] = -8000; // generate a surge at toes
    }
    Vector3d lHeel(0, 0, 0);
    Vector3d rHeel(0, 0, 0);
    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lHeel);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rHeel);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;

    // virtual force on upper body
    BodyNode *lHand = mSkel->getNode("fullbody1_h_hand_left");
    BodyNode *rHand = mSkel->getNode("fullbody1_h_hand_right");
    Vector3d lOffset =  lHand->getLocalCOM();
    Vector3d rOffset =  rHand->getLocalCOM();
    if (mPrepareFrame > 100)
        vf[1] = 1000;
    if (mPrepareFrame > 350)
        vf[1] = 3000;
    J.setZero();
    for (int i = 0; i < lHand->getNumDependentDofs(); i++) {
        int index = lHand->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lHand->getDerivWorldTransform(i), lOffset);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
        
    J.setZero();
    for (int i = 0; i < rHand->getNumDependentDofs(); i++) {
        int index = rHand->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rHand->getDerivWorldTransform(i), rOffset);
        J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
    
    // SPD control force tracking the laucnch pose
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques += p + d - mKd * qddot * mTimestep;

    // change parameters below to test different corrective torques
    if (mLaunchAng.size() > 0) {
        Vector3d lin = evalLinMomentum(_dofVel);
        Vector3d ang = evalAngMomentum(_dofVel);
        VectorXd deltaMomentum(1);
        VectorXd controlledAxis(1);
        double target = mLaunchAng[mPrepareFrame][2] * 2.0;
        deltaMomentum[0] = 1000 * (target - ang[2]);
        mOutFile << ang[2] << " "; // for the red curve
        mOutFile << mLaunchAng[mPrepareFrame][2] << " "; // for the blue curve
        mOutFile << target << endl; // for the green curve
        controlledAxis[0] = 5; // z ang momentum
        correctTorque(deltaMomentum, controlledAxis, false);
    }

    mPrepareFrame++;    
}

void Controller::computeLandingPose() {
    mProblem->getConstraints().clear();
    VectorXd pose;
    mSkel->getPose(pose);
    for (int i = 0; i < mSkel->getNumDofs(); i++)
        mProblem->vars()[i]->mVal = pose[i];
    
    // add position constraint on feet
    BodyNode *lFoot = mSkel->getNode("fullbody1_h_heel_left");
    BodyNode *rFoot = mSkel->getNode("fullbody1_h_heel_right");
    Vector3d offset = lFoot->getLocalCOM();
    Vector3d target = lFoot->getWorldCOM();
    PositionConstraint* lpos = new PositionConstraint(mProblem->vars(), mSkel, lFoot, offset, target);
    mProblem->conBox()->add(lpos);
    mProblem->getConstraints().push_back(lpos);
    target = rFoot->getWorldCOM();    
    PositionConstraint* rpos = new PositionConstraint(mProblem->vars(), mSkel, rFoot, offset, target);
    mProblem->conBox()->add(rpos);
    mProblem->getConstraints().push_back(rpos);
    
    // add COM constraint
    target = mSkel->getWorldCOM();
    target[0] += 0.15;
    COMConstraint *com = new COMConstraint(mProblem->vars(), mSkel, target);
    mProblem->conBox()->add(com);
    mProblem->getConstraints().push_back(com);
    
    mSolver->solve();
    mSkel->getPose(mLandingPose);
    mSkel->setPose(pose);

    // the optimization code for landing pose is not completed so we modify the solution a bit for now
    mLandingPose[6] = 0.5;
    mLandingPose[9] = -0.1;
    mLandingPose[10] = 0.45;
    mLandingPose[12] = -0.2;
    mLandingPose[13] = 0.5;
    mLandingPose[16] = -0.1;
    mLandingPose[17] = 0.45;
    mLandingPose[19] = -0.2;
    mLandingPose[21] = -0.6;
    mLandingPose[26] = 0.0;
    mLandingPose[28] = 0.0;
    mLandingPose[32] = 0.0;
    mLandingPose[34] = 0.0;
}

void Controller::trackLandingPose(const VectorXd& _dof, const VectorXd& _dofVel) {
    // SPD control force
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mLandingPose);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * mTimestep;

    // right now, no corrective torque is computed in airborne phase
    mControlBias.setZero();
}

void Controller::maintainBalance(const VectorXd& _dof, const VectorXd& _dofVel) {
    int nDof = mSkel->getNumDofs();
    
   // SPD control force
    mLandingPose[6] = 1.3;
    mLandingPose[9] = -2.3;
    mLandingPose[10] = 1.0;
    mLandingPose[13] = 1.3;
    mLandingPose[16] = -2.3;
    mLandingPose[17] = 1;
    mLandingPose[21] = -1;

    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mLandingPose);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * mTimestep;

    mLandingFrame++;
}
    
void Controller::detectJumpState() {
    if (mFrame < 60) {
        mJumpState = GROUND;
        return;
    }    
    int nContact = mCollisionHandle->getNumContacts();
    if (nContact == 0  && mJumpState == GROUND)
        mJumpState = TAKEOFF;
    if (nContact > 0 && mJumpState == AIRBORNE)
        mJumpState = LANDING;        
}

double Controller::computeMassTree(BodyNode *_bd) {
    if (_bd->getNumChildJoints() == 0) {
        return _bd->getMass();
    }else{
        double sum = _bd->getMass();
        for (int i = 0; i < _bd->getNumChildJoints(); i++)
            sum += computeMassTree(_bd->getChildJoint(i)->getChildNode());
        return sum;
    }
}

Vector3d Controller::evalLinMomentum(const VectorXd& _dofVel) {
    MatrixXd J(MatrixXd::Zero(3, mSkel->getNumDofs()));
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        MatrixXd localJ = node->getJacobianLinear() * node->getMass();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            J.col(index) += localJ.col(j);
        }
    }
    Vector3d cDot = J * _dofVel;
    return cDot / mSkel->getMass();
}

Vector3d Controller::evalAngMomentum(const VectorXd& _dofVel) {
    Vector3d ret = Vector3d::Zero();
    Vector3d c = mSkel->getWorldCOM();
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        Vector3d localC = node->getLocalCOM();
        Matrix4d W = node->getWorldTransform();
        Matrix4d localTranslate = Matrix4d::Identity(4, 4);
        localTranslate.block(0, 3, 3, 1) = localC;
        Matrix4d M = localTranslate * node->getShape()->getMassTensor() * localTranslate.transpose();
        Matrix4d WDot = Matrix4d::Zero();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            WDot += node->getDerivWorldTransform(j) * _dofVel[index];
        }
        Matrix3d R = W.block(0, 0, 3, 3);
        Matrix3d RDot = WDot.block(0, 0, 3, 3);
        Vector3d r = W.block(0, 3, 3, 1);
        Vector3d rDot = WDot.block(0, 3, 3, 1);

        Matrix3d rotation = R * M.block(0, 0, 3, 3) * RDot.transpose();
        Matrix3d translation = node->getMass() * r * rDot.transpose();
        Matrix3d crossTerm1 = node->getMass() * r * localC.transpose() * RDot.transpose();
        Matrix3d crossTerm2 = node->getMass() * R * localC * rDot.transpose();
        ret += utils::crossOperator(rotation + translation + crossTerm1 + crossTerm2);

        ret -= node->getMass() * c.cross(utils::xformHom(WDot, localC));
    }
    return ret;
}

void Controller::correctTorque(VectorXd _deltaMomentum, VectorXd _controlledAxis, bool _upperBodyOnly) {
    int nDof = mSkel->getNumDofs();
    double mass = mSkel->getMass();
    Matrix3d c = utils::makeSkewSymmetric(mSkel->getWorldCOM());
    MatrixXd A(MatrixXd::Zero(6, nDof));
    MatrixXd Jv(MatrixXd::Zero(3, nDof));
    MatrixXd Jw(MatrixXd::Zero(3, nDof));
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        Matrix3d c_i = utils::makeSkewSymmetric(node->getWorldCOM());
        double m_i = node->getMass();
        MatrixXd localJv = node->getJacobianLinear();
        MatrixXd localJw = node->getJacobianAngular();
        Jv.setZero();
        Jw.setZero();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int dofIndex = node->getDependentDof(j);
            Jv.col(dofIndex) += localJv.col(j);
            Jw.col(dofIndex) += localJw.col(j);
        }
        A.block(0, 0, 3, nDof) += m_i * Jv / mass;
        A.block(3, 0, 3, nDof) += m_i * (c_i - c) * Jv + node->getInertia() * Jw;
    }
    if (_upperBodyOnly) {
        for ( int i = 0; i < 18; i++)
            A.col(i).setZero();        
    } else {
        for ( int i = 0; i < 6; i++)
            A.col(i).setZero(); // try to realize momentum without using root acceleration
        for (int i = 6; i < 18; i++)
            A.col(i) *= -1; // not sure if we need to revert lower body. doesn't seem to make big diff
    }

    MatrixXd aggregateMat(_controlledAxis.size(), nDof);     
    for (int i = 0; i < _controlledAxis.size(); i++) {
        aggregateMat.row(i) = A.row(_controlledAxis[i]);
    }

    VectorXd deltaQdot = aggregateMat.transpose() * (aggregateMat * aggregateMat.transpose()).inverse() * _deltaMomentum;
    // with a regularization term
    //VectorXd deltaQdot = (aggregateMat.transpose() * aggregateMat + MatrixXd::Identity(nDof, nDof)).inverse() * aggregateMat.transpose() * _deltaMomentum;
    // make sure that this torque can be generated by valid contact forces
    //mControlBias = verifyTorque(mSkel->getMassMatrix() * deltaQdot);
    //mControlBias = mSkel->getMassMatrix() * deltaQdot;
    mControlBias = deltaQdot;

}


VectorXd Controller::verifyTorque(VectorXd _torque) {
    int nDof = mSkel->getNumDofs();
    MatrixXd A(MatrixXd::Zero(6, 6));
    MatrixXd J(MatrixXd::Zero(3, nDof));
    BodyNode *lFoot = mSkel->getNode("fullbody1_h_heel_left");
    BodyNode *rFoot = mSkel->getNode("fullbody1_h_heel_right");
    double footLen = lFoot->getShape()->getDim().maxCoeff();
    
    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), Vector3d(footLen, 0, 0));
        J.col(index) = jCol;
    }
    A.block(0, 0, 6, 3) = J.transpose().block(0, 0, 6, 3);
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), Vector3d(footLen, 0, 0));
        J.col(index) = jCol;
    }
    A.block(0, 3, 6, 3) = J.transpose().block(0, 0, 6, 3);
    VectorXd f = (A.transpose() * A + MatrixXd::Identity(6, 6)).inverse() * A.transpose() * _torque.head(6);
    Vector3d normal(0, 1, 0); // assume contact is on flat ground for now
    //    Vector3d f1 = checkContactForceValidity(f.head(3), normal, 0.785);
    //    Vector3d f2 = checkContactForceValidity(f.tail(3), normal, 0.785);
    //    f << f1, f2;
    _torque.head(6) = A * f;
    return _torque;
}

Vector3d Controller::checkContactForceValidity(Vector3d _force, Vector3d _normal, double _angle) {
    double theta = acos(_force.normalized().dot(_normal));
    if (theta <= _angle)
        return _force;
    Vector3d v = _force;
    // define new coordinate frame based on _normal
    Vector3d x_axis = Vector3d::UnitZ().cross(_normal);    
    if (x_axis.norm() < 1e-4) 
        x_axis = Vector3d::UnitX().cross(_normal);
    Vector3d z_axis = x_axis.cross(_normal);

    // new coordinates for v
    Vector3d new_v(v.dot(x_axis), v.dot(_normal), v.dot(z_axis));
    // project new_v onto the cone and scale it
    new_v[1] = sqrt(new_v[0] * new_v[0] + new_v[2] * new_v[2]) / tan(_angle);
    new_v = v.norm() * new_v.normalized();
    // rotate back to original coordinate frame
    return new_v[0] * x_axis + new_v[1] * _normal + new_v[2] * z_axis;
}

VectorXd Controller::computeVirtualTorque(Vector3d _vForce) {
    int nDof = mSkel->getNumDofs();
    MatrixXd J(MatrixXd::Zero(3, nDof));
    
    // virtual force on lower body
    BodyNode *lFoot = mSkel->getNode("fullbody1_h_heel_left");
    BodyNode *rFoot = mSkel->getNode("fullbody1_h_heel_right");
    Vector3d lSum(0, 0, 0);
    Vector3d rSum(0, 0, 0);
    int lCount = 0;
    int rCount = 0;
    
    for (int i = 0; i < mCollisionHandle->getCollisionChecker()->getNumContact(); i++) {
        if (mCollisionHandle->getCollisionChecker()->getContact(i).bd1 == lFoot || mCollisionHandle->getCollisionChecker()->getContact(i).bd2 == lFoot) {
            lSum += mCollisionHandle->getCollisionChecker()->getContact(i).point;
            lCount++;
        }
        if (mCollisionHandle->getCollisionChecker()->getContact(i).bd1 == rFoot || mCollisionHandle->getCollisionChecker()->getContact(i).bd2 == rFoot) {
            rSum += mCollisionHandle->getCollisionChecker()->getContact(i).point;
            rCount++;
        }
    }

    double footLen = lFoot->getShape()->getDim().maxCoeff();
    Vector3d lContactPt(footLen/2, 0, 0);
    Vector3d rContactPt(footLen/2, 0, 0);
    if (lCount > 0) {    
        lSum /= lCount;
        lContactPt = utils::xformHom(lFoot->getWorldTransform().inverse(), lSum);
    }
    if (rCount > 0) {
        rSum /= rCount;
        rContactPt = utils::xformHom(rFoot->getWorldTransform().inverse(), rSum); 
    }

    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lContactPt);
        J.col(index) = jCol;
    }
    VectorXd vTorque = J.transpose() * _vForce / 2.0;
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rContactPt);
        J.col(index) = jCol;
    }
    vTorque += J.transpose() * _vForce / 2.0;
    vTorque.head(6).setZero();
    return vTorque;
}    

/*
Vector3d Controller::evalAngMomentum(const VectorXd& _dofVel) {
    Vector3d com = mSkel->getWorldCOM();
    Vector3d sum = Vector3d::Zero();
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);

        Vector3d d = node->getWorldCOM() - com;
        Matrix3d Inew = node->getInertia() + node->getMass() * (d.dot(d) * MatrixXd::Identity(3, 3) - d * d.transpose());
        node->evalOmega(_dofVel);
        sum += Inew * node->mOmega;
    }
    return sum;
}
*/
>>>>>>> karen
    // PD tracking
    /*  
    for (unsigned int i = 6; i < mTorques.size(); i++) 
        mTorques[i] = -mKp(i, i) * (_dof[i] - mDesiredDofs[i])  -mKd(i, i) * _dofVel[i];

    for (int i = 6; i < nDof; i++)
        mTorques[i] *= mMassTree[i];
    */

    // ankle strategy
    /*
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");

    //Vector3d cp = (lFoot->getWorldCOM() + rFoot->getWorldCOM()) / 2.0;
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    double k1 = 10.0;
    Vector3d vf = (com - cp) * k1;
    vf[0] = -100;
    vf[1] = -mSkel->getMass() * 9.8;
    // ankle strategy
    //double k2 = 5;
    //    mDesiredDofs[10] = 0.1 - k2 * vf[0];
    //    mDesiredDofs[16] = 0.1 - k2 * vf[0];
    */
<<<<<<< HEAD

    // virtual force on lower body
    Vector3d vf(0.0, 0.0, 0.0);
    if (mFrame > 800 && mFrame < 850) {
        vf[0] = -3000;
        //vf[1] = 10000;
    }

    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    double footLen = lFoot->getShape()->getDim().maxCoeff();
    Vector3d lHeel =  utils::xformHom(lFoot->getWorldTransform(), Vector3d(footLen, 0, 0));
    Vector3d rHeel =  utils::xformHom(rFoot->getWorldTransform(), Vector3d(footLen, 0, 0));

    for (int i = 0; i < lFoot->getNumDependentDofs(); i++) {
        int index = lFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(lFoot->getDerivWorldTransform(i), lHeel);
        if (index == 9)
            J.col(index) = -jCol;
        else
            J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
        
    J.setZero();
    for (int i = 0; i < rFoot->getNumDependentDofs(); i++) {
        int index = rFoot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(rFoot->getDerivWorldTransform(i), rHeel);
        if (index == 15)
            J.col(index) = -jCol;
        else
            J.col(index) = jCol;
    }
    mTorques += J.transpose() * vf / 2.0;
=======
>>>>>>> karen
    

    // gravity compensation for upper body
    /*    
    VectorXd torque(nDof);
    torque.setZero();
    for (int i = 7; i < mSkel->getNumNodes(); i++) { // loop over each node in upperbody
        BodyNode *node = mSkel->getNode(i);
        Vector3d com = node->getLocalCOM();
        J.setZero();

        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            VectorXd jCol = utils::xformHom(node->getDerivWorldTransform(j), com);
            J.col(index) = jCol;
        }
        torque += node->getMass() * J.transpose() * Vector3d(0, 9.8, 0);
    }

    mTorques += torque;
    */

    // virtual force on COM
    /*
    Vector3d vf(0.0, 0.0, 0.0);
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    vf = (cp - com) * 1000;
    vf[0] += 18;
    vf[1] = 0.0;
    J.setZero();
    int nNode = mSkel->getNumNodes();
    for (int i = 0; i < nNode; i++) {
        BodyNode *bNode = mSkel->getNode(i);
        MatrixXd localJ = bNode->getJacobianLinear() * bNode->getMass();
        for (int j = 0; j < bNode->getNumDependentDofs(); j++) {
            int dofIndex = bNode->getDependentDof(j);
            J.col(dofIndex) += localJ.col(j);
        }
    }
    mTorques += J.transpose() / mSkel->getMass() * vf;
    */
    /*   
    // virtual force from ankles
    Vector3d vf(0.0, 0.0, 0.0);
    Vector3d com = mSkel->getWorldCOM();
    BodyNode *lFoot = mSkel->getNode("fullbody_h_foot_left");
    BodyNode *rFoot = mSkel->getNode("fullbody_h_foot_right");
    Vector3d cp = (lFoot->evalWorldPos(Vector3d::Zero()) + rFoot->evalWorldPos(Vector3d::Zero())) /2.0;
    vf = (cp - com) * 1000;
    vf[0] += 18;
    vf[1] = 0;
    if (mFrame > 800 && mFrame < 850) {
        vf[0] = 10000;
        //vf[1] = 10000;
    }
    J.setZero();
    BodyNode *node = mSkel->getNode("fullbody_h_spine");
    BodyNode *foot = mSkel->getNode("fullbody_h_foot_left");
    //    Vector3d handCOM = hand->getLocalCOM();
    //    Vector3d abdCOM = abd->getLocalCOM();
    Vector3d nodeCOM = node->getLocalCOM();
    for (int i = 0; i < foot->getNumDependentDofs(); i++) {
        int index = foot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getDerivWorldTransform(i).transpose() * node->getWorldTransform(), nodeCOM);
        if (index == 8)
            J.col(index) = -jCol;
        else
            J.col(index) = jCol;
    }
    for (int i = 0; i < node->getNumDependentDofs(); i++) {
        int index = node->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getWorldTransform().transpose() * node->getDerivWorldTransform(i), nodeCOM);
        J.col(index) = jCol;
    }
    
    //    node = mSkel->getNode("fullbody_h_scapula_right");
    foot = mSkel->getNode("fullbody_h_foot_right");
    for (int i = 0; i < foot->getNumDependentDofs(); i++) {
        int index = foot->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getDerivWorldTransform(i).transpose() * node->getWorldTransform(), nodeCOM);
        if (index == 14)
            J.col(index) -= jCol;
        else
            J.col(index) += jCol;
    }
    for (int i = 0; i < node->getNumDependentDofs(); i++) {
        int index = node->getDependentDof(i);
        VectorXd jCol = utils::xformHom(foot->getWorldTransform().transpose() * node->getDerivWorldTransform(i), nodeCOM);
        J.col(index) += jCol;
    }
    mTorques += J.transpose() * vf;
    */
<<<<<<< HEAD
    // SPD control force
    for (int i = 0; i < nDof; i++)
        mTorques[i] += -mKp(i, i) * (_dof[i] + _dofVel[i] * mTimestep - mDesiredDofs[i]) - mSkel->getKd()(i, i) * _dofVel[i];

    // Just to make sure no illegal torque is used
    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
    
    mSkel->setInternalForces(mTorques);
}

double Controller::computeMassTree(BodyNode *_bd) {
    if (_bd->getNumChildJoints() == 0) {
        return _bd->getMass();
    }else{
        double sum = _bd->getMass();
        for (int i = 0; i < _bd->getNumChildJoints(); i++)
            sum += computeMassTree(_bd->getChildJoint(i)->getChildNode());
        return sum;
    }
}

=======
>>>>>>> karen
