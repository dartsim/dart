#include "ContactDynamics.h"

#include "kinematics/BodyNode.h"
#include "lcpsolver/LCPSolver.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/collision_skeleton.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"

using namespace Eigen;
using namespace collision_checking;
using namespace utils;

namespace dynamics {
    ContactDynamics::ContactDynamics(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu, int _d)
        : mSkels(_skels), mDt(_dt), mMu(_mu), mNumDir(_d), mCollision(NULL), mAccumTime(0), mSPD(false) {
        initialize();
    }

    ContactDynamics::~ContactDynamics() {
        destroy();
    }

    void ContactDynamics::applyContactForces() {
        //        static Timer tLCP("LCP Solver");
        if (getNumTotalDofs() == 0)
            return;
        mCollision->clearAllContacts();
        mCollision->checkCollision(false);

        for (int i = 0; i < getNumSkels(); i++)
            mConstrForces[i].setZero(); 

        if (getNumContacts() == 0){
            mAccumTime = 0;
            return;
        }
        mAccumTime++;
        
        //cleanupContact();
        if (getNumContacts() > 50)
            penaltyMethod();
        else {
            fillMatrices();
            //            tLCP.startTimer();
            solve();
            //            tLCP.stopTimer();
            //            tLCP.printScreen();
            applySolution();
        }
    }

    void ContactDynamics::reset() {
        destroy();
        initialize();
    }

    void ContactDynamics::initialize() {
        // Allocate the Collision Detection class
        mCollision = new SkeletonCollision();

        mBodyIndexToSkelIndex.clear();
        // Add all body nodes into mCollision
        int rows = 0;
        int cols = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            int nNodes = skel->getNumNodes();

            for (int j = 0; j < nNodes; j++) {
                kinematics::BodyNode* node = skel->getNode(j);
                mCollision->addCollisionSkeletonNode(node);
                mBodyIndexToSkelIndex.push_back(i);
            }

            if (!mSkels[i]->getImmobileState()) {
                rows += skel->getMassMatrix().rows();
                cols += skel->getMassMatrix().cols();
            }
        }

        mConstrForces.resize(getNumSkels());
        for (int i = 0; i < getNumSkels(); i++){
            if (!mSkels[i]->getImmobileState())
                mConstrForces[i] = VectorXd::Zero(mSkels[i]->getNumDofs());
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

        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            int nDofs = skel->getNumDofs();
            if (mSkels[i]->getImmobileState())
                nDofs = 0;
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
    }

    void ContactDynamics::destroy() {
        if (mCollision) {
            delete mCollision;
        }
    }

    void ContactDynamics::updateTauStar() {
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            if (mSkels[i]->getImmobileState())
                continue;
            if (mSkels[i]->getKinematicState()) {
                mTauStar.block(startRow, 0, mSkels[i]->getNumDofs(), 1) = mSkels[i]->getQDotVector();
                startRow += mSkels[i]->getNumDofs();
                continue;
            }

            VectorXd tau = mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces();
            VectorXd tauStar;
            if (mSPD)
                tauStar = (mSkels[i]->getMassMatrix() + mDt * mSkels[i]->getKd()) * mSkels[i]->getQDotVector() - (mDt * (mSkels[i]->getCombinedVector() - tau));
            else
                tauStar = (mSkels[i]->getMassMatrix() * mSkels[i]->getQDotVector()) - (mDt * (mSkels[i]->getCombinedVector() - tau));
            mTauStar.block(startRow, 0, tauStar.rows(), 1) = tauStar;
            startRow += tauStar.rows();
        }
    }

    void ContactDynamics::updateMassMat() {
        int startRow = 0;
        int startCol = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            if (mSkels[i]->getImmobileState())
                continue;
            if (mSkels[i]->getKinematicState()) {
                int nDof = mSkels[i]->getNumDofs();
                mMInv.block(startRow, startCol, nDof, nDof) = MatrixXd::Zero(nDof, nDof);
                startRow+= nDof;
                startCol+= nDof;
                continue;
            }
            MatrixXd skelMass = mSkels[i]->getMassMatrix();
            if (mSPD)
                skelMass += mSkels[i]->getKd() * mDt;
            MatrixXd skelMassInv = skelMass.inverse();
            mMInv.block(startRow, startCol, skelMassInv.rows(), skelMassInv.cols()) = skelMassInv;
            startRow+= skelMassInv.rows();
            startCol+= skelMassInv.cols();
        }
    }
        
    void ContactDynamics::fillMatrices() {
        int c = getNumContacts();
        updateMassMat();
        updateTauStar();

        updateNormalMatrix();
        updateBasisMatrix();
        MatrixXd E = getContactMatrix();
        MatrixXd Minv = mMInv;//.inverse();
        MatrixXd mu = getMuMatrix();

        assert(Minv.rows() == Minv.cols());
        assert(Minv.cols() == mN.rows());
        assert(Minv.cols() == mB.rows());
        assert(mN.cols() == E.cols());
        assert(mB.cols() == E.rows());
        assert(mu.rows() == mu.cols());

        int dimA = c * (2 + mNumDir); // dimension of A is c + cd + c

        // Construct the intermediary blocks.
        MatrixXd Ntranspose = mN.transpose();
        MatrixXd Btranspose = mB.transpose();
        MatrixXd nTmInv = Ntranspose * Minv;
        MatrixXd bTmInv = Btranspose * Minv;
                 

        // Construct
        int cd = c * mNumDir;
        //dimA = c;
        mA = MatrixXd::Zero(dimA, dimA);
        mA.block(0, 0, c, c) = nTmInv * mN;
        mA.block(0, c, c, cd) = nTmInv * mB;
        mA.block(c, 0, cd, c) = bTmInv * mN;
        mA.block(c, c, cd, cd) = bTmInv * mB;
        mA.block(c, c + cd, cd, c) = E * (mDt * mDt);
        mA.block(c + cd, 0, c, c) = mu * (mDt * mDt);
        mA.block(c + cd, c, c, cd) = -E.transpose() * (mDt * mDt);

        int cfmSize = getNumContacts() * (1 + mNumDir);
        for (int i = 0; i < cfmSize; ++i) //add small values to diagnal to keep it away from singular, similar to cfm varaible in ODE
            mA(i, i) += 0.001 * mA(i, i);
		
        // Construct Q
        mQBar = VectorXd::Zero(dimA);

        VectorXd MinvTauStar(Minv.rows());
        int rowStart = 0;
        for (int i = 0; i < mSkels.size(); i++) {
            int nDof = mSkels[i]->getNumDofs();
            if (mSkels[i]->getImmobileState()) {
                continue;
            } else if (mSkels[i]->getKinematicState()) {
                MinvTauStar.segment(rowStart, nDof) = mTauStar.segment(rowStart, nDof);
            } else {
                MinvTauStar.segment(rowStart, nDof) = mMInv.block(rowStart, rowStart, nDof, nDof) * mTauStar.segment(rowStart, nDof);
            }
            rowStart += nDof;
        }

        mQBar.block(0, 0, c, 1) = Ntranspose * MinvTauStar;
        mQBar.block(c, 0, cd, 1) = Btranspose * MinvTauStar;

        //mQBar.block(0, 0, c, 1) = nTmInv * mTauStar;
        //mQBar.block(c, 0, cd, 1) = bTmInv * mTauStar;
        mQBar /= mDt;

        /*
        MatrixXd mat = MinvTauStar;
        for(int i = 0; i < mat.rows(); i++)
            for(int j = 0; j < mat.cols(); j++)
                //        if(mat(i, j) > 1e-6 || mat(i,j) < -1e-6)
                    cout <<"mat[" << i << "][" << j << "]= " << mat(i,j) << endl;
        */
    }

    bool ContactDynamics::solve() {
        lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
        bool b = solver.Solve(mA, mQBar, mX, mMu, mNumDir, true);
        return b;
    }
    
    void ContactDynamics::applySolution() {
        int c = getNumContacts();

        // First compute the external forces
        int nRows = mMInv.rows(); // a hacky way to get the dimension
        VectorXd forces(VectorXd::Zero(nRows));
        VectorXd f_n = mX.block(0, 0, c, 1);
        VectorXd f_d = mX.block(c, 0, c * mNumDir, 1);
        VectorXd lambda = mX.tail(c);
        forces = (mN * f_n) + (mB * f_d);

        // Next, apply the external forces skeleton by skeleton.
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            if (mSkels[i]->getImmobileState())
                continue;
            int nDof = mSkels[i]->getNumDofs();
            mConstrForces[i] = forces.block(startRow, 0, nDof, 1); 
            startRow += nDof;
        }

        for (int i = 0; i < c; i++) {
            ContactPoint& contact = mCollision->getContact(i);
            contact.force = getTangentBasisMatrix(contact.point, contact.normal) * f_d.segment(i * mNumDir, mNumDir) + contact.normal * f_n[i];
        }
            
        /*
        VectorXd sum = VectorXd::Zero(mNumDir);
        for (int i = 0; i < c; i++)
            sum += f_d.segment(i * mNumDir, mNumDir);
        double tan = sum.norm();
        if (f_n.sum() * mMu  < tan) {
            cout << "normal force = " << f_n.sum() << endl;
            cout << "tangent force = " << sum.norm() << endl;
            }*/
        //cout << "lambda = " << lambda << endl;

        /*
        MatrixXd mat = forces;
        for(int i = 0; i < mat.rows(); i++)
            for(int j = 0; j < mat.cols(); j++)
                if(mat(i, j) > 1e-6 || mat(i,j) < -1e-6)
                    cout <<"X[" << i << "][" << j << "]= " << mat(i,j) << endl;
        */
    }

    MatrixXd ContactDynamics::getJacobian(kinematics::BodyNode* node, const Vector3d& p) {
        int nDofs = node->getSkel()->getNumDofs();
        MatrixXd J( MatrixXd::Zero(3, nDofs) );
        VectorXd invP = utils::xformHom(node->getWorldInvTransform(), p); 

        for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++) {
            int i = node->getDependentDof(dofIndex);
            VectorXd Jcol = utils::xformHom(node->getDerivWorldTransform(dofIndex), (Vector3d)invP);
            J.col(i) = Jcol;
        }

        return J;
    }

    void ContactDynamics::updateNormalMatrix() {
        mN = MatrixXd::Zero(getNumTotalDofs(), getNumContacts());

        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
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
                int index2 = mIndices[skelID2];
                int NDOF2 = c.bd2->getSkel()->getNumDofs();
                Vector3d N12 = -c.normal;
                MatrixXd J12 = getJacobian(c.bd2, p);
                mN.block(index2, i, NDOF2, 1) = J12.transpose() * N12;
            }
        }
        //        N = N * mDt;        
        return;
    }

    MatrixXd ContactDynamics::getTangentBasisMatrix(const Vector3d& p, const Vector3d& n)  {
        MatrixXd T(MatrixXd::Zero(3, mNumDir));

        // Pick an arbitrary vector to take the cross product of (in this case, Z-axis)
        Vector3d tangent = Vector3d::UnitZ().cross(n);
        // If they're too close, pick another tangent (use X-axis as arbitrary vector)
        if (tangent.norm() < EPSILON) {
            tangent = Vector3d::UnitX().cross(n);
        }
        // Do we need to normalize the tangent here?
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

    void ContactDynamics::updateBasisMatrix() {
        mB = MatrixXd::Zero(getNumTotalDofs(), getNumContacts() * getNumContactDirections());
        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
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

    MatrixXd ContactDynamics::getContactMatrix() const {
        int numDir = getNumContactDirections();
        MatrixXd E(
            MatrixXd::Zero(getNumContacts() * numDir, getNumContacts())
            );
        MatrixXd column(MatrixXd::Ones(numDir, 1));
        for (int i = 0; i < getNumContacts(); i++) {
            E.block(i * numDir, i, numDir, 1) = column;
        }
        return E / (mDt * mDt);
    }

    MatrixXd ContactDynamics::getMuMatrix() const {
        int c = getNumContacts();
        // TESTING CODE BEGIN (create a frictionless node)
        /*        MatrixXd mat(MatrixXd::Identity(c, c));
        double mu;
        for (int i = 0; i < c; i++) {
            ContactPoint& contact = mCollision->getContact(i);
            if (contact.bdID1 == 0 || contact.bdID2 == 0)
                mu = 0.0;
            else
                mu = mMu;
            mat(i, i) = mu;
        }
        return mat / (mDt * mDt);
        // TESTING CODE END
        */
        return (MatrixXd::Identity(c, c) * mMu / (mDt * mDt));

    }

    int ContactDynamics::getNumContacts() const { 
        return mCollision->getNumContact(); 
    }

    void ContactDynamics::cleanupContact() {
        vector<int> deleteIDs;
        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);

            bool isUnique = true;
            for (unsigned int j = 0; j < i; j++) {
                if ((c.point - mCollision->getContact(j).point).norm() > 1e-2){
                    continue;
                }else{
                    deleteIDs.push_back(i);
                    break;
                }
            }            
        }
        //        if (deleteIDs.size() == 0)
        for (int i = deleteIDs.size() - 1; i >= 0; i--) {
            mCollision->mContactPointList.erase(mCollision->mContactPointList.begin() + deleteIDs[i]);
        }

        int nNode = mCollision->mCollisionSkeletonNodeList.size();
        /*
          // choose three most important contact points per pair of bodies
        vector<vector<int> > contactGroups;
        contactGroups.resize(nNode * (nNode + 1) / 2);
        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
            int bd1 = c.bdID1;
            int bd2 = c.bdID2;
            int row = bd1;
            int col = bd2;
            if(bd1 > bd2) {
                row = bd2;
                col = bd1;
            }
            int key = nNode * row - row * (row - 1) / 2 + col - row;
            //            cout << row << " " << col << " " << key << endl;
            contactGroups[key].push_back(i);
        }
        for (int i = 0; i < contactGroups.size(); i++) {
            if (contactGroups[i].size() < 4)
                continue;
            ContactPoint& a = mCollision->getContact(contactGroups[i][0]);
            ContactPoint& b = mCollision->getContact(contactGroups[i][1]);
            Vector3d v = b.point - a.point;
            double currMax = 0.0;
            int pick = 2;
            for (int j = 2; j < contactGroups[i].size(); j++) {
                Vector3d p = mCollision->getContact(contactGroups[i][j]).point;
                double t = (p - a.point).dot(v) / v.dot(v);
                Vector3d diff = p - a.point + t * v;
                double dist = diff.dot(diff);
                if (dist > currMax)
                    pick = j;
            }
            cout << pick << endl;
            for (int j = contactGroups[i].size() - 1; j >= 2; j--) {
                if (j == pick)
                    continue;                
                mCollision->mContactPointList.erase(mCollision->mContactPointList.begin()+ contactGroups[i][j]);
            }
            }*/

        // limite to 4 points per pair of bodies
        //        int nNode = mCollision->mCollisionSkeletonNodeList.size();
              MatrixXd pointCount(nNode, nNode);
        pointCount.setZero();
        for (int i = getNumContacts() - 1; i >= 0; i--) {
            ContactPoint& c = mCollision->getContact(i);
            int bd1 = c.bdID1;
            int bd2 = c.bdID2;
            if (bd1 < bd2) {
                pointCount(bd2, bd1)++;
                if (pointCount(bd2, bd1) > 4)
                    mCollision->mContactPointList.erase(mCollision->mContactPointList.begin() + i);
            } else {
                pointCount(bd1, bd2)++;
                if (pointCount(bd1, bd2) > 4)
                    mCollision->mContactPointList.erase(mCollision->mContactPointList.begin() + i);
            }
        }
        //        cout << pointCount << endl;
    }
    
    void ContactDynamics::penaltyMethod() {
        int nContact = getNumContacts();
        for (int i = 0; i < nContact; i++) {
            ContactPoint& c = mCollision->getContact(i);
            double d = c.penetrationDepth;
            Vector3d f = d * c.normal;
            Vector3d p = c.point;
            int sID1 = mBodyIndexToSkelIndex[c.bdID1];
            int sID2 = mBodyIndexToSkelIndex[c.bdID2];           

            int nDof = mSkels[sID1]->getNumDofs();
            MatrixXd J(MatrixXd::Zero(3, nDof));
            Vector3d invP = utils::xformHom(c.bd1->getWorldInvTransform(), p);
            VectorXd qDot = mSkels[sID1]->getQDotVector();
            double ks = 200 * mSkels[sID1]->getMass();
            double kd = 100 * mSkels[sID1]->getMass();
            if (!mSkels[sID1]->getImmobileState()){
                for (int j = 0; j < c.bd1->getNumDependentDofs(); j++) {
                    VectorXd Jcol = utils::xformHom(c.bd1->getDerivWorldTransform(j), invP);
                    J.col(c.bd1->getDependentDof(j)) = Jcol;
                }
                Vector3d v = J * qDot;
                Vector3d vN = v.dot(c.normal) * c.normal;
                Vector3d vT = v - vN;
                Vector3d fN = (f * ks - vN * kd) / nContact;
                Vector3d fT;
                if( vT.norm() < 1e-7)
                    fT.setZero();
                else
                    fT = fN.norm() * vT.normalized();
                if( fN.dot(c.normal) < 0)
                    fN.setZero();
                VectorXd Q = J.transpose() * (fN - 1 * fT);
                mConstrForces[sID1] += Q;
            }
            
            nDof = mSkels[sID2]->getNumDofs();
            J = MatrixXd::Zero(3, nDof);
            invP = utils::xformHom(c.bd2->getWorldInvTransform(), p); 
            qDot = mSkels[sID2]->getQDotVector();
            ks = 200 * mSkels[sID2]->getMass();
            kd = 100 * mSkels[sID2]->getMass();
            double ki = 2 * mSkels[sID2]->getMass();
            if (!mSkels[sID2]->getImmobileState()){
                for (int j = 0; j < c.bd2->getNumDependentDofs(); j++) {
                    VectorXd Jcol = utils::xformHom(c.bd2->getDerivWorldTransform(j), invP);
                    J.col(c.bd2->getDependentDof(j)) = Jcol;
                }
                Vector3d v = J * qDot;
                Vector3d vN = v.dot(c.normal) * c.normal;
                Vector3d vT = v - vN;
                Vector3d fN = (-f * ks - vN * kd - ki * mAccumTime * f) / nContact;
                Vector3d fT;
                if( vT.norm() < 1e-7)
                    fT.setZero();
                else
                    fT = fN.norm() * vT.normalized();
                if( fN.dot(-c.normal) < 0)
                    fN.setZero();
                VectorXd Q = J.transpose() * (fN - 1 * fT);
                cout << "contact " << i << " " << mAccumTime << endl;
                mConstrForces[sID2] += Q;
            }
        }
        cout << "sum " << mConstrForces[1][1]  << endl;

    }
}
