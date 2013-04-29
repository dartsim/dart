
#include "ConstraintDynamics.h"

#include "kinematics/BodyNode.h"
#include "lcpsolver/LCPSolver.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/CollisionSkeleton.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"

using namespace Eigen;
using namespace collision_checking;
using namespace utils;

    namespace dynamics {
        ConstraintDynamics::ConstraintDynamics(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu, int _d)
            : mSkels(_skels), mDt(_dt), mMu(_mu), mNumDir(_d), mCollisionChecker(NULL) {
            initialize();
        }

        ConstraintDynamics::~ConstraintDynamics() {
            if (mCollisionChecker)
                delete mCollisionChecker;
        }

        void ConstraintDynamics::computeConstraintForces() {
            if (getTotalNumDofs() == 0)
                return;
            mCollisionChecker->clearAllContacts();
            mCollisionChecker->checkCollision(true, true);

            if (mCollisionChecker->getNumContact() == 0) {
                for (int i = 0; i < mSkels.size(); i++)
                    mContactForces[i].setZero();
                if (mConstraints.size() == 0) {
                    for (int i = 0; i < mSkels.size(); i++)
                        mTotalConstrForces[i].setZero();
                } else {
                    computeConstraintWithoutContact();
                }
            } else {
                fillMatrices();
                solve();
                applySolution();
            }
        }

        void ConstraintDynamics::initialize() {
            // Allocate the Collision Detection class
            mCollisionChecker = new SkeletonCollision();

            mBodyIndexToSkelIndex.clear();
            // Add all body nodes into mCollisionChecker
            int rows = 0;
            int cols = 0;
            for (int i = 0; i < mSkels.size(); i++) {
                SkeletonDynamics* skel = mSkels[i];
                int nNodes = skel->getNumNodes();

                for (int j = 0; j < nNodes; j++) {
                    kinematics::BodyNode* node = skel->getNode(j);
                    if(node->getCollideState()) {
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
                mContactForces[i] = VectorXd::Zero(mSkels[i]->getNumDofs());
                mTotalConstrForces[i] = VectorXd::Zero(mSkels[i]->getNumDofs());
            }

            mMInv = MatrixXd::Zero(rows, cols);
            mTauStar = VectorXd::Zero(rows);

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
                SkeletonDynamics* skel = mSkels[i];
                int nDofs = skel->getNumDofs();
                if (mSkels[i]->getImmobileState())
                    nDofs = 0;
                sumNDofs += nDofs;
                mIndices.push_back(sumNDofs);
            }

            mTotalRows = 0;
            mJ.resize(mSkels.size());
            mPreJ.resize(mSkels.size());
            mJMInv.resize(mSkels.size());
            mZ = MatrixXd(rows, cols);
        }

        void ConstraintDynamics::updateTauStar() {
            int startRow = 0;
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;

                VectorXd tau = mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces();
                VectorXd tauStar = (mSkels[i]->getMassMatrix() * mSkels[i]->getPoseVelocity()) - (mDt * (mSkels[i]->getCombinedVector() - tau));
                mTauStar.block(startRow, 0, tauStar.rows(), 1) = tauStar;
                startRow += tauStar.rows();
            }
        }

        void ConstraintDynamics::updateMassMat() {
            int startRow = 0;
            int startCol = 0;
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                MatrixXd skelMassInv = mSkels[i]->getInvMassMatrix();
                mMInv.block(startRow, startCol, skelMassInv.rows(), skelMassInv.cols()) = skelMassInv;
                startRow+= skelMassInv.rows();
                startCol+= skelMassInv.cols();
            }
        }

        void ConstraintDynamics::fillMatrices() {
            updateMassMat();
            updateTauStar();

            updateNBMatrices();
            if (mConstraints.size() > 0)
                updateConstraintTerms();

            MatrixXd E = getContactMatrix();
            MatrixXd mu = getMuMatrix();

            // Construct the intermediary blocks.
            MatrixXd Ntranspose = mN.transpose();
            MatrixXd Btranspose = mB.transpose();
            // Compute NTerm and BTerm
            MatrixXd tempMat = mMInv;
            if (mConstraints.size() > 0)
                tempMat -= mZ;
            MatrixXd NTerm = tempMat * mN;
            MatrixXd BTerm = tempMat * mB;

            // Construct
            int c = getNumContacts();
            int cd = c * mNumDir;
            int dimA = c * (2 + mNumDir); // dimension of A is c + cd + c
            mA = MatrixXd::Zero(dimA, dimA);
            mA.block(0, 0, c, c) = Ntranspose * NTerm;
            mA.block(0, c, c, cd) = Ntranspose * BTerm;
            mA.block(c, 0, cd, c) = Btranspose * NTerm;
            mA.block(c, c, cd, cd) = Btranspose * BTerm;
            mA.block(c, c + cd, cd, c) = E;
            mA.block(c + cd, 0, c, c) = mu;
            mA.block(c + cd, c, c, cd) = -E.transpose();

            int cfmSize = getNumContacts() * (1 + mNumDir);
            for (int i = 0; i < cfmSize; ++i) //add small values to diagnal to keep it away from singular, similar to cfm varaible in ODE
                mA(i, i) += 0.001 * mA(i, i);

            // Construct Q
            mQBar = VectorXd::Zero(dimA);
            VectorXd tauVec = VectorXd::Zero(getTotalNumDofs());
            if (mConstraints.size() > 0) {
                VectorXd tempVec = mDt * mGInv * mTauHat;
                for (int i = 0; i < mSkels.size(); i++) {
                    if (mSkels[i]->getImmobileState())
                        continue;
                    tauVec.segment(mIndices[i], mSkels[i]->getNumDofs()) = mJ[i].transpose() * tempVec;
                }
            }
            tauVec = mMInv * (tauVec + mTauStar);
            mQBar.block(0, 0, c, 1) = Ntranspose * tauVec;
            mQBar.block(c, 0, cd, 1) = Btranspose * tauVec;
            mQBar /= mDt;
        }

        bool ConstraintDynamics::solve() {
            lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
            bool b = solver.Solve(mA, mQBar, mX, mMu, mNumDir, true);
            return b;
        }

        void ConstraintDynamics::applySolution() {
            // First compute the external forces
            int nRows = mMInv.rows(); // a hacky way to get the dimension
            VectorXd forces(VectorXd::Zero(nRows));
            VectorXd f_n = mX.block(0, 0, getNumContacts(), 1);
            VectorXd f_d = mX.block(getNumContacts(), 0, getNumContacts() * mNumDir, 1);
            forces = (mN * f_n) + (mB * f_d);

            VectorXd lambda = VectorXd::Zero(mGInv.rows());
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                mContactForces[i] = forces.segment(mIndices[i], mSkels[i]->getNumDofs());

                mTotalConstrForces[i] = mContactForces[i];
                if (mConstraints.size() > 0) {
                    VectorXd tempVec = mGInv * (mTauHat - mJMInv[i] * forces.segment(mIndices[i], mSkels[i]->getNumDofs()));                    
                    mTotalConstrForces[i] += mJ[i].transpose() * tempVec;
                    lambda += tempVec;
                }
            }
            int count = 0;
            for (int i = 0; i < mConstraints.size(); i++) {
                mConstraints[i]->setLagrangeMultipliers(lambda.segment(count, mConstraints[i]->getNumRows()));
                count += mConstraints[i]->getNumRows();
            }

            for (int i = 0; i < getNumContacts(); i++) {
                ContactPoint& contact = mCollisionChecker->getContact(i);
                contact.force = getTangentBasisMatrix(contact.point, contact.normal) * f_d.segment(i * mNumDir, mNumDir) + contact.normal * f_n[i];
            }
        }

        MatrixXd ConstraintDynamics::getJacobian(kinematics::BodyNode* node, const Vector3d& p) {
            int nDofs = node->getSkel()->getNumDofs();
            MatrixXd Jt( MatrixXd::Zero(nDofs, 3) );
            VectorXd invP = utils::xformHom(node->getWorldInvTransform(), p);

            for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++) {
                int i = node->getDependentDof(dofIndex);
                VectorXd Jcol = utils::xformHom(node->getDerivWorldTransform(dofIndex), (Vector3d)invP);
                Jt.row(i) = Jcol;
            }

            return Jt;
        }

        void ConstraintDynamics::updateNBMatrices() {
            mN = MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
            mB = MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * mNumDir);
            for (int i = 0; i < getNumContacts(); i++) {
                ContactPoint& c = mCollisionChecker->getContact(i);
                Vector3d p = c.point;
                int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
                int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

                Vector3d N21 = c.normal;
                Vector3d N12 = -c.normal;
                MatrixXd B21 = getTangentBasisMatrix(p, N21);
                MatrixXd B12 = -B21;

                if (!mSkels[skelID1]->getImmobileState()) {
                    int index1 = mIndices[skelID1];
                    int NDOF1 = c.bd1->getSkel()->getNumDofs();
                    //    Vector3d N21 = c.normal;
                    MatrixXd J21t = getJacobian(c.bd1, p);
                    mN.block(index1, i, NDOF1, 1) = J21t * N21;
                    //B21 = getTangentBasisMatrix(p, N21);
                    mB.block(index1, i * mNumDir, NDOF1, mNumDir) = J21t * B21;
                }

                if (!mSkels[skelID2]->getImmobileState()) {
                    int index2 = mIndices[skelID2];
                    int NDOF2 = c.bd2->getSkel()->getNumDofs();
                    //Vector3d N12 = -c.normal;
                    //if (B21.rows() == 0)
                    //  B12 = getTangentBasisMatrix(p, N12);
                    //else
                    //   B12 = -B21;
                    MatrixXd J12t = getJacobian(c.bd2, p);
                    mN.block(index2, i, NDOF2, 1) = J12t * N12;
                    mB.block(index2, i * mNumDir, NDOF2, mNumDir) = J12t * B12;

                }
            }
        }
        MatrixXd ConstraintDynamics::getTangentBasisMatrix(const Vector3d& p, const Vector3d& n)  {
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
                T.block(0, i, 3, 1) = basis;

                if (mNumDir % 2 == 0) {
                    T.block(0, i + iter, 3, 1) = -basis;
                }
            }
            return T;
        }
        MatrixXd ConstraintDynamics::getContactMatrix() const {
            MatrixXd E(MatrixXd::Zero(getNumContacts() * mNumDir, getNumContacts()));
            MatrixXd column(MatrixXd::Ones(mNumDir, 1));
            for (int i = 0; i < getNumContacts(); i++) {
                E.block(i * mNumDir, i, mNumDir, 1) = column;
            }
            return E;
        }

        MatrixXd ConstraintDynamics::getMuMatrix() const {
            int c = getNumContacts();
            return MatrixXd::Identity(c, c) * mMu;

        }

        void ConstraintDynamics::updateConstraintTerms(){
            // update preJ
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                mPreJ[i] = mJ[i];
            }
            // compute J
            int count = 0;
            for (int i = 0; i < mConstraints.size(); i++) {
                mConstraints[i]->updateDynamics(mJ, mC, mCDot, count);
                count += mConstraints[i]->getNumRows();
            }
            // compute JMInv, GInv, Z
            mGInv.setZero();
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                mJMInv[i] = mJ[i] * mSkels[i]->getInvMassMatrix();
                mGInv += (mJMInv[i] * mJ[i].transpose());
            }
            mGInv = mGInv.inverse();
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;

                for (int j = 0; j < mSkels.size(); j++) {
                    if (mSkels[j]->getImmobileState())
                        continue;
                    mZ.block(mIndices[i], mIndices[j], mSkels[i]->getNumDofs(), mSkels[j]->getNumDofs()) = mJMInv[i].transpose() * mGInv * mJMInv[j];
                }
            }

            // compute tauHat
            double ks = 100;
            double kd = 10;
            mTauHat.setZero();
            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                VectorXd qDot = mSkels[i]->getPoseVelocity();
                mTauHat += -(mJ[i] - mPreJ[i]) / mDt * qDot - mJMInv[i] * (mSkels[i]->getInternalForces() + mSkels[i]->getExternalForces() - mSkels[i]->getCombinedVector());
            }
            mTauHat -= ks * mC + kd * mCDot;
        }

        void ConstraintDynamics::addConstraint(Constraint *_constr) {
            mConstraints.push_back(_constr);
            mTotalRows += _constr->getNumRows();
            for (int i = 0; i < mSkels.size(); i++) {
                mJ[i].conservativeResize(mTotalRows, mSkels[i]->getNumDofs());
                mJ[i].bottomRows(_constr->getNumRows()).setZero();
            } 
            mC = VectorXd(mTotalRows);
            mCDot = VectorXd(mTotalRows);
            mGInv = MatrixXd::Zero(mTotalRows, mTotalRows);
            mTauHat = VectorXd(mTotalRows);
        }

        void ConstraintDynamics::deleteConstraint(int _index) {
            int count = 0;
            for (int i = 0; i < _index; i++)
                count += mConstraints[i]->getNumRows();
            int shiftRows = mTotalRows - count - mConstraints[_index]->getNumRows();
            mTotalRows -= mConstraints[_index]->getNumRows();

            for (int i = 0; i < mSkels.size(); i++) {
                mJ[i].block(count, 0, shiftRows, mSkels[i]->getNumDofs()) = mJ[i].bottomRows(shiftRows);
                mJ[i].conservativeResize(mTotalRows, mSkels[i]->getNumDofs());
            } 
            mC.resize(mTotalRows);
            mCDot.resize(mTotalRows);
            mGInv.resize(mTotalRows, mTotalRows);
            mTauHat.resize(mTotalRows);

            mConstraints.erase(mConstraints.begin() + _index);
            delete mConstraints[_index];
        }
        
        void ConstraintDynamics::computeConstraintWithoutContact() {
            updateMassMat();
            updateConstraintTerms();
            VectorXd lambda = mGInv * mTauHat;

            for (int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState())
                    continue;
                mTotalConstrForces[i] = mJ[i].transpose() * lambda;                
            }
        }
    }

