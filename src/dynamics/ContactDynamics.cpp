#include "ContactDynamics.h"

#include "kinematics/BodyNode.h"
#include "lcpsolver/LCPSolver.h"
#include "kinematics/Shape.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/fcl/FCLCollisionDetector.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"
#include "math/UtilsMath.h"
#include "utils/Timer.h"

using namespace Eigen;
using namespace collision;
using namespace utils;

#define EPSILON 0.000001

namespace dynamics {

ContactDynamics::ContactDynamics(const std::vector<SkeletonDynamics*>& _skels,
                                 double _dt,
                                 double _mu,
                                 int _d)
    : mSkels(_skels),
      mDt(_dt),
      mMu(_mu),
      mNumDir(_d),
      mCollisionChecker(NULL) {
    initialize();
}

ContactDynamics::~ContactDynamics() {
    destroy();
}

void ContactDynamics::applyContactForces() {
    if (getNumTotalDofs() == 0)
        return;

    mCollisionChecker->clearAllContacts();
    mCollisionChecker->checkCollision(true, true);

    for (int i = 0; i < getNumSkels(); i++)
        mConstrForces[i].setZero();

    if (mCollisionChecker->getNumContacts() == 0)
        return;

    fillMatrices();
    solve();
    applySolution();
}

void ContactDynamics::reset() {
    destroy();
    initialize();
}

void ContactDynamics::initialize() {
    // Allocate the Collision Detection class
    //mCollisionChecker = new FCLCollisionDetector();
    mCollisionChecker = new FCLMESHCollisionDetector();

    mBodyIndexToSkelIndex.clear();
    // Add all body nodes into mCollisionChecker
    for (int i = 0; i < getNumSkels(); i++) {
        SkeletonDynamics* skel = mSkels[i];
        int nNodes = skel->getNumNodes();

        for (int j = 0; j < nNodes; j++) {
            mCollisionChecker->addCollisionSkeletonNode(skel->getNode(j));
            mBodyIndexToSkelIndex.push_back(i);
        }
    }

    mConstrForces.resize(getNumSkels());
    for (int i = 0; i < getNumSkels(); i++){
        if (!mSkels[i]->getImmobileState())
            mConstrForces[i] = VectorXd::Zero(mSkels[i]->getNumDofs());
    }

    mIndices.clear();
    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);
    for (int i = 0; i < getNumSkels(); i++) {
        if (!mSkels[i]->getImmobileState())
            sumNDofs += mSkels[i]->getNumDofs();
        mIndices.push_back(sumNDofs);
    }

    mTauStar = VectorXd::Zero(getNumTotalDofs());
}

void ContactDynamics::addSkeleton(SkeletonDynamics* _newSkel) {
    //
    mSkels.push_back(_newSkel);

    int nSkels = mSkels.size();
    int nNodes = _newSkel->getNumNodes();

    for (int j = 0; j < nNodes; j++) {
        mCollisionChecker->addCollisionSkeletonNode(_newSkel->getNode(j));
        mBodyIndexToSkelIndex.push_back(nSkels-1);
    }

    Eigen::VectorXd newConstrForce;
    if (!_newSkel->getImmobileState())
        newConstrForce = VectorXd::Zero(_newSkel->getNumDofs());
    mConstrForces.push_back(newConstrForce);

    // TODO: World already has this indices.
    // Initialize the index vector:
    // If we have 3 skeletons,
    // mIndices[0] = 0
    // mIndices[1] = nDof0
    // mIndices[2] = nDof0 + nDof1
    // mIndices[3] = nDof0 + nDof1 + nDof2
    mIndices.clear();
    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);
    for (int i = 0; i < getNumSkels(); i++) {
        if (!mSkels[i]->getImmobileState())
            sumNDofs += mSkels[i]->getNumDofs();
        mIndices.push_back(sumNDofs);
    }

    mTauStar = VectorXd::Zero(getNumTotalDofs());
}

void ContactDynamics::destroy() {
    if (mCollisionChecker) {
        delete mCollisionChecker;
    }
}

void ContactDynamics::updateTauStar() {
    int startRow = 0;
    for (int i = 0; i < getNumSkels(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;

        VectorXd tau = mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces();
        VectorXd tauStar = mSkels[i]->getMassMatrix() * mSkels[i]->get_dq();
        tauStar.noalias() -= (mDt * (mSkels[i]->getCombinedVector() - tau));
        mTauStar.segment(startRow, tauStar.rows()) = tauStar;
        startRow += tauStar.rows();
    }
}

void ContactDynamics::fillMatrices() {
    updateTauStar();

    updateNBMatrices();
    //        updateNormalMatrix();
    //        updateBasisMatrix();

    MatrixXd E = getContactMatrix();

    int c = getNumContacts();
    int cd = c * mNumDir;

    // Construct the intermediary blocks.
    // nTmInv = mN.transpose() * MInv
    // bTmInv = mB.transpose() * MInv
    // Where MInv is the imaginary diagonal block matrix that combines the inverted mass matrices of all skeletons.
    // Muliplying each block independently is more efficient that multiplyting the whole MInv matrix.
    MatrixXd nTmInv(c, getNumTotalDofs());
    MatrixXd bTmInv(cd, getNumTotalDofs());
    for (int i = 0; i < getNumSkels(); i++) {
        if (mSkels[i]->getImmobileState()) {
            assert(mIndices[i] == mIndices[i+1]); // If the user sets a skeleton to be immobile without reinitializing ContactDynamics, this assertion will fail.
            continue;
        }
        const MatrixXd skelMInv = mSkels[i]->getInvMassMatrix();
        const int skelNumDofs = mSkels[i]->getNumDofs();
        nTmInv.middleCols(mIndices[i], skelNumDofs).noalias() = mN.transpose().middleCols(mIndices[i], skelNumDofs) * skelMInv;
        bTmInv.middleCols(mIndices[i], skelNumDofs).noalias() = mB.transpose().middleCols(mIndices[i], skelNumDofs) * skelMInv;
    }

    // Construct
    int dimA = c * (2 + mNumDir); // dimension of A is c + cd + c
    mA.resize(dimA, dimA);
    mA.topLeftCorner(c, c).triangularView<Upper>() = nTmInv * mN;
    mA.topLeftCorner(c, c).triangularView<StrictlyLower>() = mA.topLeftCorner(c, c).transpose();
    mA.block(0, c, c, cd).noalias() = nTmInv * mB;
    mA.block(c, 0, cd, c) = mA.block(0, c, c, cd).transpose(); // since B^T * Minv * N = (N^T * Minv * B)^T
    mA.block(c, c, cd, cd).triangularView<Upper>() = bTmInv * mB;
    mA.block(c, c, cd, cd).triangularView<StrictlyLower>() = mA.block(c, c, cd, cd).transpose();
    //        mA.block(c, c + cd, cd, c) = E * (mDt * mDt);
    mA.block(c, c + cd, cd, c) = E;
    //        mA.block(c + cd, 0, c, c) = mu * (mDt * mDt);
    mA.bottomLeftCorner(c, c) = getMuMatrix(); // Note: mu is a diagonal matrix, but we also set the surrounding zeros
    //        mA.block(c + cd, c, c, cd) = -E.transpose() * (mDt * mDt);
    mA.block(c + cd, c, c, cd) = -E.transpose();
    mA.topRightCorner(c, c).setZero();
    mA.bottomRightCorner(c, c).setZero();

    int cfmSize = getNumContacts() * (1 + mNumDir);
    for (int i = 0; i < cfmSize; ++i) //add small values to diagnal to keep it away from singular, similar to cfm varaible in ODE
        mA(i, i) += 0.001 * mA(i, i);

    // Construct Q
    mQBar = VectorXd::Zero(dimA);

    /*
            VectorXd MinvTauStar(mN.rows());
            int rowStart = 0;
            for (int i = 0; i < mSkels.size(); i++) {
                int nDof = mSkels[i]->getNumDofs();
                if (mSkels[i]->getImmobileState()) {
                    continue;
                } else {
                    MinvTauStar.segment(rowStart, nDof) = mMInv.block(rowStart, rowStart, nDof, nDof) * mTauStar.segment(rowStart, nDof);
                }
                rowStart += nDof;
            }
            */
    //mQBar.block(0, 0, c, 1) = mN.transpose() * MinvTauStar;
    //mQBar.block(c, 0, cd, 1) = mB.transpose() * MinvTauStar;

    mQBar.head(c).noalias() = nTmInv * mTauStar;
    mQBar.segment(c,cd).noalias() = bTmInv * mTauStar;
    mQBar /= mDt;
}

bool ContactDynamics::solve() {
    lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
    bool b = solver.Solve(mA, mQBar, mX, getNumContacts(), mMu, mNumDir, true);
    return b;
}

void ContactDynamics::applySolution() {
    const int c = getNumContacts();

    // First compute the external forces
    VectorXd f_n = mX.head(c);
    VectorXd f_d = mX.segment(c, c * mNumDir);
    VectorXd lambda = mX.tail(c);
    VectorXd forces = mN * f_n;
    forces.noalias() += mB * f_d;

    // Next, apply the external forces skeleton by skeleton.
    int startRow = 0;
    for (int i = 0; i < getNumSkels(); i++) {
        if (mSkels[i]->getImmobileState())
            continue;
        int nDof = mSkels[i]->getNumDofs();
        mConstrForces[i] = forces.segment(startRow, nDof);
        startRow += nDof;
    }

    for (int i = 0; i < c; i++) {
        Contact& contact = mCollisionChecker->getContact(i);
        contact.force.noalias() = getTangentBasisMatrix(contact.point, contact.normal) * f_d.segment(i * mNumDir, mNumDir);
        contact.force += contact.normal * f_n[i];
    }
}

MatrixXd ContactDynamics::getJacobian(kinematics::BodyNode* node, const Vector3d& p) {
    int nDofs = node->getSkel()->getNumDofs();
    MatrixXd Jt( MatrixXd::Zero(nDofs, 3) );
    Vector3d invP = dart_math::xformHom(node->getWorldInvTransform(), p);

    for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++) {
        int i = node->getDependentDof(dofIndex);
        Jt.row(i) = dart_math::xformHom(node->getDerivWorldTransform(dofIndex), invP);
    }

    return Jt;
}

void ContactDynamics::updateNBMatrices() {
    mN = MatrixXd::Zero(getNumTotalDofs(), getNumContacts());
    mB = MatrixXd::Zero(getNumTotalDofs(), getNumContacts() * getNumContactDirections());
    for (int i = 0; i < getNumContacts(); i++) {
        Contact& c = mCollisionChecker->getContact(i);
        Vector3d p = c.point;
        int skelID1 = mBodyIndexToSkelIndex[c.collisionNode1->getIndex()];
        int skelID2 = mBodyIndexToSkelIndex[c.collisionNode2->getIndex()];

        Vector3d N21 = c.normal;
        Vector3d N12 = -c.normal;
        MatrixXd B21 = getTangentBasisMatrix(p, N21);
        MatrixXd B12 = -B21;

        if (!mSkels[skelID1]->getImmobileState()) {
            int index1 = mIndices[skelID1];
            int NDOF1 = c.collisionNode1->getBodyNode()->getSkel()->getNumDofs();
            //    Vector3d N21 = c.normal;
            MatrixXd J21t = getJacobian(c.collisionNode1->getBodyNode(), p);
            mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
            //B21 = getTangentBasisMatrix(p, N21);
            mB.block(index1, i * getNumContactDirections(), NDOF1, getNumContactDirections()).noalias() = J21t * B21;
        }

        if (!mSkels[skelID2]->getImmobileState()) {
            int index2 = mIndices[skelID2];
            int NDOF2 = c.collisionNode2->getBodyNode()->getSkel()->getNumDofs();
            //Vector3d N12 = -c.normal;
            //if (B21.rows() == 0)
            //  B12 = getTangentBasisMatrix(p, N12);
            //else
            //   B12 = -B21;
            MatrixXd J12t = getJacobian(c.collisionNode2->getBodyNode(), p);
            mN.block(index2, i, NDOF2, 1).noalias() = J12t * N12;
            mB.block(index2, i * getNumContactDirections(), NDOF2, getNumContactDirections()).noalias() = J12t * B12;

        }
    }
}
/*
        void ContactDynamics::updateNormalMatrix() {
            static Timer t1("t1");
            static Timer t2("t2");
            static Timer t3("t3");

            mN = MatrixXd::Zero(getNumTotalDofs(), getNumContacts());
            for (int i = 0; i < getNumContacts(); i++) {
                ContactPoint& c = mCollisionChecker->getContact(i);
                Vector3d p = c.point;
                int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
                int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

                if (!mSkels[skelID1]->getImmobileState()) {
                    int index1 = mIndices[skelID1];
                    int NDOF1 = c.bd1->getSkel()->getNumDofs();
                    Vector3d N21 = c.normal;
                    MatrixXd J21 = getJacobian(c.bd1, p);
                    mN.block(index1, i, NDOF1, 1) = J21.transpose() * N21;
                }
                if (!mSkels[skelID2]->getImmobileState()) {
                    t1.startTimer();
                    int index2 = mIndices[skelID2];
                    int NDOF2 = c.bd2->getSkel()->getNumDofs();
                    Vector3d N12 = -c.normal;
                    int nDofs = c.bd2->getSkel()->getNumDofs();
                    MatrixXd J12( MatrixXd::Zero(3, nDofs) );
                    VectorXd invP = math::xformHom(c.bd2->getWorldInvTransform(), p);

                    t2.startTimer();
                    for(int dofIndex = 0; dofIndex < c.bd2->getNumDependentDofs(); dofIndex++) {
                        int index = c.bd2->getDependentDof(dofIndex);
                        VectorXd Jcol = math::xformHom(c.bd2->getDerivWorldTransform(dofIndex), (Vector3d)invP);
                        J12.col(index) = Jcol;
                    }
                    t2.stopTimer();
                    if (t2.getCount() == 1500)
                        t2.printScreen();
                    //                MatrixXd J12 = getJacobian(c.bd2, p);
                    mN.block(index2, i, NDOF2, 1) = J12.transpose() * N12;

                    t1.stopTimer();
                    if (t1.getCount() == 1500) {
                        t1.printScreen();
                        cout << t2.getAve() / t1.getAve() * 100 << "%" << endl;
                    }
                }

            }
            return;
        }
        */
MatrixXd ContactDynamics::getTangentBasisMatrix(const Vector3d& p, const Vector3d& n)  {
    MatrixXd T(MatrixXd::Zero(3, mNumDir));

    // Pick an arbitrary vector to take the cross product of (in this case, Z-axis)
    Vector3d tangent = Vector3d::UnitZ().cross(n);
    // If they're too close, pick another tangent (use X-axis as arbitrary vector)
    if (tangent.norm() < EPSILON) {
        tangent = Vector3d::UnitX().cross(n);
    }
    tangent.normalize();

    // Rotate the tangent around the normal to compute bases.
    // Note: a possible speedup is in place for mNumDir % 2 = 0
    // Each basis and its opposite belong in the matrix, so we iterate half as many times.
    double angle = (2 * M_PI) / mNumDir;
    int iter = (mNumDir % 2 == 0) ? mNumDir / 2 : mNumDir;
    for (int i = 0; i < iter; i++) {
        Vector3d basis = Quaterniond(AngleAxisd(i * angle, n)) * tangent;
        T.block<3,1>(0, i) = basis;

        if (mNumDir % 2 == 0) {
            T.block<3,1>(0, i + iter) = -basis;
        }
    }
    return T;
}
/*
        void ContactDynamics::updateBasisMatrix() {
            mB = MatrixXd::Zero(getNumTotalDofs(), getNumContacts() * getNumContactDirections());
            for (int i = 0; i < getNumContacts(); i++) {
                ContactPoint& c = mCollisionChecker->getContact(i);
                Vector3d p = c.point;
                int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
                int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

                MatrixXd B21;
                MatrixXd B12;
                if (!mSkels[skelID1]->getImmobileState()) {
                    int index1 = mIndices[skelID1];
                    int NDOF1 = c.bd1->getSkel()->getNumDofs();
                    B21 = getTangentBasisMatrix(p, c.normal);
                    MatrixXd J21 = getJacobian(c.bd1, p);

                    mB.block(index1, i * getNumContactDirections(), NDOF1, getNumContactDirections()) = J21.transpose() * B21;

                    //  cout << "B21: " << B21 << endl;
                }

                if (!mSkels[skelID2]->getImmobileState()) {
                    int index2 = mIndices[skelID2];
                    int NDOF2 = c.bd2->getSkel()->getNumDofs();
                    if (B21.rows() == 0)
                        B12 = getTangentBasisMatrix(p, -c.normal);
                    else
                        B12 = -B21;
                    MatrixXd J12 = getJacobian(c.bd2, p);
                    mB.block(index2, i * getNumContactDirections(), NDOF2, getNumContactDirections()) = J12.transpose() * B12;

                    //  cout << "B12: " << B12 << endl;

                }
            }
            return;
        }
        */
MatrixXd ContactDynamics::getContactMatrix() const {
    int numDir = getNumContactDirections();
    MatrixXd E(MatrixXd::Zero(getNumContacts() * numDir, getNumContacts()));
    MatrixXd column(MatrixXd::Ones(numDir, 1));
    for (int i = 0; i < getNumContacts(); i++) {
        E.block(i * numDir, i, numDir, 1) = column;
    }
    //        return E / (mDt * mDt);
    return E;
}

MatrixXd ContactDynamics::getMuMatrix() const {
    int c = getNumContacts();
    // TESTING CODE BEGIN (create a frictionless node)
    /*        MatrixXd mat(MatrixXd::Identity(c, c));
            double mu;
            for (int i = 0; i < c; i++) {
                ContactPoint& contact = mCollisionChecker->getContact(i);
                if (contact.bdID1 == 0 || contact.bdID2 == 0)
                    mu = 0.0;
                else
                    mu = mMu;
                mat(i, i) = mu;
            }
            return mat / (mDt * mDt);
            // TESTING CODE END
            */
    //        return (MatrixXd::Identity(c, c) * mMu / (mDt * mDt));
    return MatrixXd::Identity(c, c) * mMu;

}

int ContactDynamics::getNumContacts() const {
    return mCollisionChecker->getNumContacts();
}

} // namespace dynamics
