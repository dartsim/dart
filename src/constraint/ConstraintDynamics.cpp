#include "constraint/ConstraintDynamics.h"

#include "common/Timer.h"
#include "common/UtilsCode.h"
#include "math/Helpers.h"
#include "lcpsolver/LCPSolver.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "collision/fcl/FCLCollisionDetector.h"
#include "dynamics/BodyNode.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Skeleton.h"

using namespace dart;
using namespace collision;
using namespace math;

#define EPSILON 0.000001

namespace dart {
namespace constraint {

    ConstraintDynamics::ConstraintDynamics(const std::vector<dynamics::Skeleton*>& _skels, double _dt, double _mu, int _d, bool _useODE)
        : mSkels(_skels), mDt(_dt), mMu(_mu), mNumDir(_d), mCollisionChecker(NULL), mUseODELCPSolver(_useODE) {
    initialize();
}

ConstraintDynamics::~ConstraintDynamics() {
    if (mCollisionChecker)
        delete mCollisionChecker;
}

void ConstraintDynamics::reset() {
    destroy();
    initialize();
}

void ConstraintDynamics::computeConstraintForces() {
    //            static Timer t1("t1");

    if (getTotalNumDofs() == 0)
        return;
    mCollisionChecker->clearAllContacts();
    mCollisionChecker->checkCollision(true, true);

    //            t1.startTimer();
    mLimitingDofIndex.clear();

    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState() || !mSkels[i]->getJointLimitState())
            continue;
        for (int j = 0; j < mSkels[i]->getDOF(); j++) {
            double val = mSkels[i]->getGenCoord(j)->get_q();
            double ub = mSkels[i]->getGenCoord(j)->get_qMax();
            double lb = mSkels[i]->getGenCoord(j)->get_qMin();
            if (val >= ub){
                mLimitingDofIndex.push_back(mIndices[i] + j + 1);
                //        cout << "Skeleton " << i << " Dof " << j << " hits upper bound" << endl;
            }
            if (val <= lb){
                mLimitingDofIndex.push_back(-(mIndices[i] + j + 1));
                //      cout << "Skeleton " << i << " Dof " << j << " hits lower bound" << endl;
            }
        }
    }

    if (mCollisionChecker->getNumContacts() == 0 && mLimitingDofIndex.size() == 0) {
        for (int i = 0; i < mSkels.size(); i++)
            mContactForces[i].setZero();
        if (mConstraints.size() == 0) {
            for (int i = 0; i < mSkels.size(); i++)
                mTotalConstrForces[i].setZero();
        } else {
            computeConstraintWithoutContact();
        }
    } else {
        if (mUseODELCPSolver) {
            fillMatricesODE();
            solve();
            applySolutionODE();
        } else {
            fillMatrices();
            solve();
            applySolution();            
        }
    }
    //            t1.stopTimer();
    //            t1.printScreen();
}

void ConstraintDynamics::addConstraint(Constraint *_constr) {
    mConstraints.push_back(_constr);
    mTotalRows += _constr->getNumRows();
    for (int i = 0; i < mSkels.size(); i++) {
        mJ[i].conservativeResize(mTotalRows, mSkels[i]->getDOF());
        mJ[i].bottomRows(_constr->getNumRows()).setZero();
    }
    mC = Eigen::VectorXd(mTotalRows);
    mCDot = Eigen::VectorXd(mTotalRows);
    mGInv = Eigen::MatrixXd::Zero(mTotalRows, mTotalRows);
    mTauHat = Eigen::VectorXd(mTotalRows);
}

void ConstraintDynamics::deleteConstraint(int _index) {
    int count = 0;
    for (int i = 0; i < _index; i++)
        count += mConstraints[i]->getNumRows();
    int shiftRows = mTotalRows - count - mConstraints[_index]->getNumRows();
    mTotalRows -= mConstraints[_index]->getNumRows();

    for (int i = 0; i < mSkels.size(); i++) {
        mJ[i].block(count, 0, shiftRows, mSkels[i]->getDOF()) = mJ[i].bottomRows(shiftRows);
        mJ[i].conservativeResize(mTotalRows, mSkels[i]->getDOF());
    }
    mC.resize(mTotalRows);
    mCDot.resize(mTotalRows);
    mGInv.resize(mTotalRows, mTotalRows);
    mTauHat.resize(mTotalRows);

    mConstraints.erase(mConstraints.begin() + _index);
    delete mConstraints[_index];
}

void ConstraintDynamics::addSkeleton(dynamics::Skeleton* _newSkel)
{
    mSkels.push_back(_newSkel);

    int nSkels = mSkels.size();
    int nNodes = _newSkel->getNumBodyNodes();

    for (int j = 0; j < nNodes; j++) {
        mCollisionChecker->addCollisionSkeletonNode(_newSkel->getBodyNode(j));
        mBodyIndexToSkelIndex.push_back(nSkels-1);
    }

    // Add all body nodes into mCollisionChecker
    int rows = mMInv.rows();
    int cols = mMInv.cols();

    if (!_newSkel->getImmobileState()) {
        // Immobile objets have mass of infinity
        rows += _newSkel->getMassMatrix().rows();
        cols += _newSkel->getMassMatrix().cols();
    }

    Eigen::VectorXd newConstrForce;
    if (!_newSkel->getImmobileState())
        newConstrForce = Eigen::VectorXd::Zero(_newSkel->getDOF());
    mContactForces.push_back(newConstrForce);
    mTotalConstrForces.push_back(newConstrForce);

    mMInv = Eigen::MatrixXd::Zero(rows, cols);
    mTauStar = Eigen::VectorXd::Zero(rows);

    mIndices.clear();
    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);

    for (int i = 0; i < mSkels.size(); i++) {
        dynamics::Skeleton* skel = mSkels[i];
        int nDofs = skel->getDOF();
        if (mSkels[i]->getImmobileState())
            nDofs = 0;
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
    }
    mJ.resize(mSkels.size());
    mPreJ.resize(mSkels.size());
    mJMInv.resize(mSkels.size());
    mZ = Eigen::MatrixXd(rows, cols);
}

void ConstraintDynamics::initialize() {
    // Allocate the Collision Detection class
    //mCollisionChecker = new FCLCollisionDetector();
    mCollisionChecker = new FCLMESHCollisionDetector();
    mBodyIndexToSkelIndex.clear();
    // Add all body nodes into mCollisionChecker
    int rows = 0;
    int cols = 0;
    for (int i = 0; i < mSkels.size(); i++) {
        dynamics::Skeleton* skel = mSkels[i];
        int nNodes = skel->getNumBodyNodes();

        for (int j = 0; j < nNodes; j++) {
            dynamics::BodyNode* node = skel->getBodyNode(j);
            if(node->isCollidable()) {
                mCollisionChecker->addCollisionSkeletonNode(node);
                mBodyIndexToSkelIndex.push_back(i);
            }
        }

        if (!mSkels[i]->getImmobileState()) {
            // Immobile objets have mass of infinity
            rows += skel->getMassMatrix().rows();
            cols += skel->getMassMatrix().cols();
        }
    }

    mContactForces.resize(mSkels.size());
    mTotalConstrForces.resize(mSkels.size());
    for (int i = 0; i < mSkels.size(); i++){
        if (mSkels[i]->getImmobileState())
            continue;
        mContactForces[i] = Eigen::VectorXd::Zero(mSkels[i]->getDOF());
        mTotalConstrForces[i] = Eigen::VectorXd::Zero(mSkels[i]->getDOF());
    }

    mMInv = Eigen::MatrixXd::Zero(rows, cols);
    mTauStar = Eigen::VectorXd::Zero(rows);

    // Initialize the index vector:
    // If we have 3 skeletons,
    // mIndices[0] = 0
    // mIndices[1] = nDof0
    // mIndices[2] = nDof0 + nDof1
    // mIndices[3] = nDof0 + nDof1 + nDof2

    mIndices.clear();
    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);

    for (int i = 0; i < mSkels.size(); i++) {
        dynamics::Skeleton* skel = mSkels[i];
        int nDofs = skel->getDOF();
        if (mSkels[i]->getImmobileState())
            nDofs = 0;
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
    }

    mTotalRows = 0;
    mJ.resize(mSkels.size());
    mPreJ.resize(mSkels.size());
    mJMInv.resize(mSkels.size());
    mZ = Eigen::MatrixXd(rows, cols);
}

void ConstraintDynamics::destroy()
{
    if (mCollisionChecker)
        delete mCollisionChecker;
}

void ConstraintDynamics::computeConstraintWithoutContact() {
    updateMassMat();
    updateConstraintTerms();
    Eigen::VectorXd lambda = mGInv * mTauHat;

    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mTotalConstrForces[i] = mJ[i].transpose() * lambda;
    }
}

void ConstraintDynamics::fillMatrices() {
    int nContacts = getNumContacts();
    int nJointLimits = mLimitingDofIndex.size();
    int nConstrs = mConstraints.size();
    int cd = nContacts * mNumDir;
    int dimA = nContacts * (2 + mNumDir) + nJointLimits;
    mA = Eigen::MatrixXd::Zero(dimA, dimA);
    mQBar = Eigen::VectorXd::Zero(dimA);
    updateMassMat();
    updateTauStar();

    Eigen::MatrixXd augMInv = mMInv;
    Eigen::VectorXd tauVec = Eigen::VectorXd::Zero(getTotalNumDofs());
    if (nConstrs > 0) {
        updateConstraintTerms();
        augMInv -= mZ.triangularView<Eigen::Lower>();

        Eigen::VectorXd tempVec = mDt * mGInv * mTauHat;
        for (int i = 0; i < mSkels.size(); i++) {
            if (mSkels[i]->getImmobileState())
                continue;
            tauVec.segment(mIndices[i], mSkels[i]->getDOF()) = mJ[i].transpose() * tempVec;
        }
    }
    tauVec = mMInv * (tauVec + mTauStar);

    Eigen::MatrixXd NTerm(getTotalNumDofs(), nContacts);
    Eigen::MatrixXd BTerm(getTotalNumDofs(), cd);

    if (nContacts > 0) {
        updateNBMatrices();
        Eigen::MatrixXd E = getContactMatrix();
        // Compute NTerm and BTerm
        NTerm.noalias() = augMInv * mN;
        BTerm.noalias() = augMInv * mB;
        mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::Lower>() = mN.transpose() * NTerm;
        mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::StrictlyUpper>() = mA.block(0, 0, nContacts, nContacts).transpose();
        mA.block(0, nContacts, nContacts, cd).noalias() = mN.transpose() * BTerm;
        mA.block(nContacts, 0, cd, nContacts) = mA.block(0, nContacts, nContacts, cd).transpose();
        mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::Lower>() = mB.transpose() * BTerm;
        mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::StrictlyUpper>() = mA.block(nContacts, nContacts, cd, cd).transpose();
        mA.block(nContacts, nContacts + cd, cd, nContacts) = E;
        mA.block(nContacts + cd, 0, nContacts, nContacts) = getMuMatrix();
        mA.block(nContacts + cd, nContacts, nContacts, cd) = -E.transpose();

        mQBar.segment(0, nContacts).noalias() = mN.transpose() * tauVec;
        mQBar.segment(nContacts, cd).noalias() = mB.transpose() * tauVec;
    }

    if (nJointLimits > 0) {
        int jointStart = 2 * nContacts + cd;
        for (int i = 0; i < nJointLimits; i++)
            for (int j = 0; j < nJointLimits; j++) {
                if (mLimitingDofIndex[i] * mLimitingDofIndex[j] < 0)
                    mA(jointStart + i, jointStart + j) = -augMInv(abs(mLimitingDofIndex[i]) - 1, abs(mLimitingDofIndex[j]) - 1);
                else
                    mA(jointStart + i, jointStart + j) = augMInv(abs(mLimitingDofIndex[i]) - 1, abs(mLimitingDofIndex[j]) - 1);
            }

        for (int i = 0; i < nJointLimits; i++) {
            if (mLimitingDofIndex[i] > 0) // hitting upper bound
                mQBar[jointStart + i] = -tauVec[abs(mLimitingDofIndex[i]) - 1];
            else // hitting lower bound
                mQBar[jointStart + i] = tauVec[abs(mLimitingDofIndex[i]) - 1];
        }

        if (nContacts > 0) {

            Eigen::MatrixXd STerm(mMInv.rows(), nJointLimits);
            for (int i = 0; i < nJointLimits; i++) {
                if (mLimitingDofIndex[i] > 0) // hitting upper bound
                    STerm.col(i) = -augMInv.col(mLimitingDofIndex[i] - 1);
                else
                    STerm.col(i) = augMInv.col(abs(mLimitingDofIndex[i]) - 1);
            }
            mA.block(0, jointStart, nContacts, nJointLimits) = mN.transpose() * STerm;

            mA.block(nContacts, jointStart, cd, nJointLimits) = mB.transpose() * STerm;

            for (int i = 0; i < nJointLimits; i++) {
                if (mLimitingDofIndex[i] > 0) { //hitting uppder bound
                    mA.block(jointStart + i, 0, 1, nContacts) = -NTerm.row(mLimitingDofIndex[i] - 1);
                    mA.block(jointStart + i, nContacts, 1, cd) = -BTerm.row(mLimitingDofIndex[i] - 1);
                } else {
                    mA.block(jointStart + i, 0, 1, nContacts) = NTerm.row(abs(mLimitingDofIndex[i]) - 1);
                    mA.block(jointStart + i, nContacts, 1, cd) = BTerm.row(abs(mLimitingDofIndex[i]) - 1);
                }
            }

        }
    }
    mQBar /= mDt;

    int cfmSize = getNumContacts() * (1 + mNumDir);
    for (int i = 0; i < cfmSize; ++i) //add small values to diagnal to keep it away from singular, similar to cfm varaible in ODE
        mA(i, i) += 0.001 * mA(i, i);
}

void ConstraintDynamics::fillMatricesODE() {
    int nContacts = getNumContacts();
    int nJointLimits = mLimitingDofIndex.size();
    int nConstrs = mConstraints.size();
    int cd = nContacts * 2;
    int dimA = nContacts * 3 + nJointLimits;
    mA = Eigen::MatrixXd(dimA, dimA);
    mQBar = Eigen::VectorXd(dimA);
    updateMassMat();
    updateTauStar();

    Eigen::MatrixXd augMInv = mMInv;
    Eigen::VectorXd tauVec = Eigen::VectorXd::Zero(getTotalNumDofs());
    if (nConstrs > 0) {
        updateConstraintTerms();
        augMInv -= mZ.triangularView<Eigen::Lower>();

        Eigen::VectorXd tempVec = mDt * mGInv * mTauHat;
        for (int i = 0; i < mSkels.size(); i++) {
            if (mSkels[i]->getImmobileState())
                continue;
            tauVec.segment(mIndices[i], mSkels[i]->getDOF()) = mJ[i].transpose() * tempVec;
        }
    }
    tauVec = mMInv * (tauVec + mTauStar);

    Eigen::MatrixXd NTerm(getTotalNumDofs(), nContacts);
    Eigen::MatrixXd BTerm(getTotalNumDofs(), cd);

    if (nContacts > 0) {
        updateNBMatricesODE();
        // Compute NTerm and BTerm
        NTerm.noalias() = augMInv * mN;
        BTerm.noalias() = augMInv * mB;
        mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::Lower>() = mN.transpose() * NTerm;
        mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::StrictlyUpper>() = mA.block(0, 0, nContacts, nContacts).transpose();
        mA.block(0, nContacts, nContacts, cd).noalias() = mN.transpose() * BTerm;
        mA.block(nContacts, 0, cd, nContacts) = mA.block(0, nContacts, nContacts, cd).transpose();
        mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::Lower>() = mB.transpose() * BTerm;
        mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::StrictlyUpper>() = mA.block(nContacts, nContacts, cd, cd).transpose();

        mQBar.segment(0, nContacts).noalias() = mN.transpose() * tauVec;
        mQBar.segment(nContacts, cd).noalias() = mB.transpose() * tauVec;
    }

    if (nJointLimits > 0) {
        int jointStart = nContacts + cd;
        for (int i = 0; i < nJointLimits; i++)
            for (int j = 0; j < nJointLimits; j++) {
                if (mLimitingDofIndex[i] * mLimitingDofIndex[j] < 0)
                    mA(jointStart + i, jointStart + j) = -augMInv(abs(mLimitingDofIndex[i]) - 1, abs(mLimitingDofIndex[j]) - 1);
                else
                    mA(jointStart + i, jointStart + j) = augMInv(abs(mLimitingDofIndex[i]) - 1, abs(mLimitingDofIndex[j]) - 1);
            }

        for (int i = 0; i < nJointLimits; i++) {
            if (mLimitingDofIndex[i] > 0) // hitting upper bound
                mQBar[jointStart + i] = -tauVec[abs(mLimitingDofIndex[i]) - 1];
            else // hitting lower bound
                mQBar[jointStart + i] = tauVec[abs(mLimitingDofIndex[i]) - 1];
        }

        if (nContacts > 0) {

            Eigen::MatrixXd STerm(mMInv.rows(), nJointLimits);
            for (int i = 0; i < nJointLimits; i++) {
                if (mLimitingDofIndex[i] > 0) // hitting upper bound
                    STerm.col(i) = -augMInv.col(mLimitingDofIndex[i] - 1);
                else
                    STerm.col(i) = augMInv.col(abs(mLimitingDofIndex[i]) - 1);
            }
            mA.block(0, jointStart, nContacts, nJointLimits) = mN.transpose() * STerm;

            mA.block(nContacts, jointStart, cd, nJointLimits) = mB.transpose() * STerm;

            for (int i = 0; i < nJointLimits; i++) {
                if (mLimitingDofIndex[i] > 0) { //hitting uppder bound
                    mA.block(jointStart + i, 0, 1, nContacts) = -NTerm.row(mLimitingDofIndex[i] - 1);
                    mA.block(jointStart + i, nContacts, 1, cd) = -BTerm.row(mLimitingDofIndex[i] - 1);
                } else {
                    mA.block(jointStart + i, 0, 1, nContacts) = NTerm.row(abs(mLimitingDofIndex[i]) - 1);
                    mA.block(jointStart + i, nContacts, 1, cd) = BTerm.row(abs(mLimitingDofIndex[i]) - 1);
                }
            }

        }
    }
    mQBar /= mDt;

    int cfmSize = getNumContacts() * 3;
    for (int i = 0; i < cfmSize; ++i) //add small values to diagnal to keep it away from singular, similar to cfm varaible in ODE
        mA(i, i) += 0.001 * mA(i, i);
}


bool ConstraintDynamics::solve() {
    lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
    bool b = solver.Solve(mA, mQBar, mX, getNumContacts(), mMu, mNumDir, mUseODELCPSolver);
    return b;
}

void ConstraintDynamics::applySolution() {
    Eigen::VectorXd contactForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
    Eigen::VectorXd jointLimitForces(Eigen::VectorXd::Zero(getTotalNumDofs()));

    if (getNumContacts() > 0) {
        Eigen::VectorXd f_n = mX.head(getNumContacts());
        Eigen::VectorXd f_d = mX.segment(getNumContacts(), getNumContacts() * mNumDir);
        contactForces.noalias() = mN * f_n;
        contactForces.noalias() += mB * f_d;
        for (int i = 0; i < getNumContacts(); i++) {
            Contact& contact = mCollisionChecker->getContact(i);
            contact.force.noalias() = getTangentBasisMatrix(contact.point, contact.normal) * f_d.segment(i * mNumDir, mNumDir);
            contact.force.noalias() += contact.normal * f_n[i];
        }
    }
    for (int i = 0; i < mLimitingDofIndex.size(); i++) {
        if (mLimitingDofIndex[i] > 0) { // hitting upper bound
            jointLimitForces[mLimitingDofIndex[i] - 1] = -mX[getNumContacts() * (2 + mNumDir) + i];
        }else{
            jointLimitForces[abs(mLimitingDofIndex[i]) - 1] = mX[getNumContacts() * (2 + mNumDir) + i];
        }
    }

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(mGInv.rows());
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mContactForces[i] = contactForces.segment(mIndices[i], mSkels[i]->getDOF());

        mTotalConstrForces[i] = mContactForces[i] + jointLimitForces.segment(mIndices[i], mSkels[i]->getDOF());

        if (mConstraints.size() > 0) {
            Eigen::VectorXd tempVec = mGInv * (mTauHat - mJMInv[i] * (contactForces.segment(mIndices[i], mSkels[i]->getDOF()) + jointLimitForces.segment(mIndices[i], mSkels[i]->getDOF())));
            mTotalConstrForces[i] += mJ[i].transpose() * tempVec;
            lambda += tempVec;
        }
    }

    int count = 0;
    for (int i = 0; i < mConstraints.size(); i++) {
        mConstraints[i]->setLagrangeMultipliers(lambda.segment(count, mConstraints[i]->getNumRows()));
        count += mConstraints[i]->getNumRows();
    }

}

void ConstraintDynamics::applySolutionODE() {
    Eigen::VectorXd contactForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
    Eigen::VectorXd jointLimitForces(Eigen::VectorXd::Zero(getTotalNumDofs()));

    if (getNumContacts() > 0) {
        Eigen::VectorXd f_n = mX.head(getNumContacts());
        Eigen::VectorXd f_d = mX.segment(getNumContacts(), getNumContacts() * 2);
        contactForces.noalias() = mN * f_n;
        contactForces.noalias() += mB * f_d;
        for (int i = 0; i < getNumContacts(); i++) {
            Contact& contact = mCollisionChecker->getContact(i);
            contact.force.noalias() = getTangentBasisMatrixODE(contact.point, contact.normal) * f_d.segment(i * 2, 2);
            contact.force.noalias() += contact.normal * f_n[i];
        }
    }
    for (int i = 0; i < mLimitingDofIndex.size(); i++) {
        if (mLimitingDofIndex[i] > 0) { // hitting upper bound
            jointLimitForces[mLimitingDofIndex[i] - 1] = -mX[getNumContacts() * 3 + i];
        }else{
            jointLimitForces[abs(mLimitingDofIndex[i]) - 1] = mX[getNumContacts() * 3 + i];
        }
    }

    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(mGInv.rows());
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mContactForces[i] = contactForces.segment(mIndices[i], mSkels[i]->getDOF());

        mTotalConstrForces[i] = mContactForces[i] + jointLimitForces.segment(mIndices[i], mSkels[i]->getDOF());

        if (mConstraints.size() > 0) {
            Eigen::VectorXd tempVec = mGInv * (mTauHat - mJMInv[i] * (contactForces.segment(mIndices[i], mSkels[i]->getDOF()) + jointLimitForces.segment(mIndices[i], mSkels[i]->getDOF())));
            mTotalConstrForces[i] += mJ[i].transpose() * tempVec;
            lambda += tempVec;
        }
    }

    int count = 0;
    for (int i = 0; i < mConstraints.size(); i++) {
        mConstraints[i]->setLagrangeMultipliers(lambda.segment(count, mConstraints[i]->getNumRows()));
        count += mConstraints[i]->getNumRows();
    }

}


void ConstraintDynamics::updateMassMat() {
    int start = 0;
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mMInv.block(start, start, mSkels[i]->getDOF(), mSkels[i]->getDOF()) = mSkels[i]->getInvMassMatrix();
        start += mSkels[i]->getDOF();
    }
}

void ConstraintDynamics::updateTauStar() {
    int startRow = 0;
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;

        Eigen::VectorXd tau = mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces() + mSkels[i]->getDampingForces();
        Eigen::VectorXd tauStar = (mSkels[i]->getMassMatrix() * mSkels[i]->get_dq()) - (mDt * (mSkels[i]->getCombinedVector() - tau));
        mTauStar.block(startRow, 0, tauStar.rows(), 1) = tauStar;
        startRow += tauStar.rows();
    }
}

void ConstraintDynamics::updateNBMatrices() {
    mN = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
    mB = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * mNumDir);
    for (int i = 0; i < getNumContacts(); i++) {
        Contact& c = mCollisionChecker->getContact(i);
        Eigen::Vector3d p = c.point;
        int skelID1 = mBodyIndexToSkelIndex[c.collisionNode1->getIndex()];
        int skelID2 = mBodyIndexToSkelIndex[c.collisionNode2->getIndex()];

        Eigen::Vector3d N21 = c.normal;
        Eigen::Vector3d N12 = -c.normal;
        Eigen::MatrixXd B21 = getTangentBasisMatrix(p, N21);
        Eigen::MatrixXd B12 = -B21;

        if (!mSkels[skelID1]->getImmobileState()) {
            int index1 = mIndices[skelID1];
            int NDOF1 = c.collisionNode1->getBodyNode()->getSkeleton()->getDOF();
            //    Vector3d N21 = c.normal;
            Eigen::MatrixXd J21t = getJacobian(c.collisionNode1->getBodyNode(), p);
            mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
            //B21 = getTangentBasisMatrix(p, N21);
            mB.block(index1, i * mNumDir, NDOF1, mNumDir).noalias() = J21t * B21;
        }

        if (!mSkels[skelID2]->getImmobileState()) {
            int index2 = mIndices[skelID2];
            int NDOF2 = c.collisionNode2->getBodyNode()->getSkeleton()->getDOF();
            //Vector3d N12 = -c.normal;
            //if (B21.rows() == 0)
            //  B12 = getTangentBasisMatrix(p, N12);
            //else
            //   B12 = -B21;
            Eigen::MatrixXd J12t = getJacobian(c.collisionNode2->getBodyNode(), p);
            mN.block(index2, i, NDOF2, 1).noalias() = J12t * N12;
            mB.block(index2, i * mNumDir, NDOF2, mNumDir).noalias() = J12t * B12;

        }
    }
}

void ConstraintDynamics::updateNBMatricesODE() {
    mN = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
    mB = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * 2);
    for (int i = 0; i < getNumContacts(); i++) {
        Contact& c = mCollisionChecker->getContact(i);
        Eigen::Vector3d p = c.point;
        int skelID1 = mBodyIndexToSkelIndex[c.collisionNode1->getIndex()];
        int skelID2 = mBodyIndexToSkelIndex[c.collisionNode2->getIndex()];

        Eigen::Vector3d N21 = c.normal;
        Eigen::Vector3d N12 = -c.normal;
        Eigen::MatrixXd B21 = getTangentBasisMatrixODE(p, N21);
        Eigen::MatrixXd B12 = -B21;

        if (!mSkels[skelID1]->getImmobileState()) {
            int index1 = mIndices[skelID1];
            int NDOF1 = c.collisionNode1->getBodyNode()->getSkeleton()->getDOF();
            //    Vector3d N21 = c.normal;
            Eigen::MatrixXd J21t = getJacobian(c.collisionNode1->getBodyNode(), p);
            mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
            //B21 = getTangentBasisMatrix(p, N21);
            mB.block(index1, i * 2, NDOF1, 2).noalias() = J21t * B21;
        }

        if (!mSkels[skelID2]->getImmobileState()) {
            int index2 = mIndices[skelID2];
            int NDOF2 = c.collisionNode2->getBodyNode()->getSkeleton()->getDOF();
            //Vector3d N12 = -c.normal;
            //if (B21.rows() == 0)
            //  B12 = getTangentBasisMatrix(p, N12);
            //else
            //   B12 = -B21;
            Eigen::MatrixXd J12t = getJacobian(c.collisionNode2->getBodyNode(), p);
            mN.block(index2, i, NDOF2, 1).noalias() = J12t * N12;
            mB.block(index2, i * 2, NDOF2, 2).noalias() = J12t * B12;

        }
    }
}

Eigen::MatrixXd ConstraintDynamics::getJacobian(dynamics::BodyNode* node, const Eigen::Vector3d& p) {
    int nDofs = node->getSkeleton()->getDOF();
    Eigen::MatrixXd Jt = Eigen::MatrixXd::Zero(nDofs, 3);

    Eigen::MatrixXd JtBody
            = node->getWorldJacobianAtPoint_LinearPartOnly(p).transpose();

    for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++)
    {
        int i = node->getDependentDof(dofIndex);
        Jt.row(i) = JtBody.row(dofIndex);
    }

    return Jt;
}

Eigen::MatrixXd ConstraintDynamics::getTangentBasisMatrix(const Eigen::Vector3d& p, const Eigen::Vector3d& n)  {
    Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, mNumDir));

    // Pick an arbitrary vector to take the cross product of (in this case, Z-axis)
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(n);
    // If they're too close, pick another tangent (use X-axis as arbitrary vector)
    if (tangent.norm() < EPSILON) {
        tangent = Eigen::Vector3d::UnitX().cross(n);
    }
    tangent.normalize();

    // Rotate the tangent around the normal to compute bases.
    // Note: a possible speedup is in place for mNumDir % 2 = 0
    // Each basis and its opposite belong in the matrix, so we iterate half as many times.
    double angle = (2 * DART_PI) / mNumDir;
    int iter = (mNumDir % 2 == 0) ? mNumDir / 2 : mNumDir;
    for (int i = 0; i < iter; i++) {
        Eigen::Vector3d basis = Eigen::Quaterniond(Eigen::AngleAxisd(i * angle, n)) * tangent;
        T.col(i) = basis;

        if (mNumDir % 2 == 0) {
            T.col(i + iter) = -basis;
        }
    }
    return T;
}

Eigen::MatrixXd ConstraintDynamics::getTangentBasisMatrixODE(const Eigen::Vector3d& p, const Eigen::Vector3d& n)  {
    Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, 2));

    // Pick an arbitrary vector to take the cross product of (in this case, Z-axis)
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(n);
    // If they're too close, pick another tangent (use X-axis as arbitrary vector)
    if (tangent.norm() < EPSILON) {
        tangent = Eigen::Vector3d::UnitX().cross(n);
    }
    tangent.normalize();

    // Rotate the tangent around the normal to compute bases.
    // Note: a possible speedup is in place for mNumDir % 2 = 0
    // Each basis and its opposite belong in the matrix, so we iterate half as many times

    double angle = DART_PI / 2.0;
    T.col(0) = tangent;
    T.col(1) = Eigen::Quaterniond(Eigen::AngleAxisd(angle, n)) * tangent;
    return T;
}


Eigen::MatrixXd ConstraintDynamics::getContactMatrix() const {
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(getNumContacts() * mNumDir, getNumContacts());
    Eigen::VectorXd column = Eigen::VectorXd::Ones(mNumDir);
    for (int i = 0; i < getNumContacts(); i++) {
        E.block(i * mNumDir, i, mNumDir, 1) = column;
    }
    return E;
}

Eigen::MatrixXd ConstraintDynamics::getMuMatrix() const {
    int c = getNumContacts();
    return Eigen::MatrixXd::Identity(c, c) * mMu;

}

void ConstraintDynamics::updateConstraintTerms(){
    // update preJ
    mPreJ = mJ;
    // compute J
    int count = 0;
    for (int i = 0; i < mConstraints.size(); i++) {
        mConstraints[i]->updateDynamics(mJ, mC, mCDot, count);
        count += mConstraints[i]->getNumRows();
    }
    // compute JMInv, GInv, Z
    mGInv.triangularView<Eigen::Lower>().setZero();
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mJMInv[i] = mJ[i] * mSkels[i]->getInvMassMatrix();
        mGInv.triangularView<Eigen::Lower>() += (mJMInv[i] * mJ[i].transpose());
    }
    mGInv = mGInv.ldlt().solve(Eigen::MatrixXd::Identity(mTotalRows, mTotalRows));
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        mZ.block(mIndices[i], mIndices[i], mSkels[i]->getDOF(), mSkels[i]->getDOF()).triangularView<Eigen::Lower>() = mJMInv[i].transpose() * mGInv * mJMInv[i];
        for (int j = 0; j < i; j++) {
            if (mSkels[j]->getImmobileState())
                continue;
            mZ.block(mIndices[i], mIndices[j], mSkels[i]->getDOF(), mSkels[j]->getDOF()).noalias() = mJMInv[i].transpose() * mGInv * mJMInv[j];
        }
    }

    // compute tauHat
    double ks = 500;
    double kd = 50;
    mTauHat.setZero();
    for (int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        Eigen::VectorXd qDot = mSkels[i]->get_dq();
        mTauHat.noalias() += -(mJ[i] - mPreJ[i]) / mDt * qDot;
        mTauHat.noalias() -= mJMInv[i] * (mSkels[i]->getInternalForces() + mSkels[i]->getExternalForces() - mSkels[i]->getCombinedVector());
    }
    mTauHat -= ks * mC + kd * mCDot;
}
}
}
