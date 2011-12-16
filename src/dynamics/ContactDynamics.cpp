#include "ContactDynamics.h"

#include "kinematics/BodyNode.h"
#include "lcpsolver/LCPSolver.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/collision_skeleton.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace collision_checking;

namespace dynamics {
    ContactDynamic::ContactDynamic(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu, int _d)
        : mSkels(_skels), mDt(_dt), mMu(_mu), mNumDir(_d), mCollision(NULL) {
        initialize();
    }

    ContactDynamic::~ContactDynamic() {
        destroy();
    }

    void ContactDynamic::computeTauStar() {
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            VectorXd tau = mSkels[i]->getExternalForces();
            VectorXd tauStar = (mSkels[i]->getMassMatrix() * mSkels[i]->getQDotVector()) - (mDt * (mSkels[i]->getCombinedVector() - tau));
            mTauStar.block(startRow, 0, tauStar.rows(), 1) = tauStar;
            startRow += tauStar.rows();
        }
    }

    void ContactDynamic::computeMassMat() {
        int startRow = 0;
        int startCol = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            MatrixXd skelMass = mSkels[i]->getMassMatrix();
            mM.block(startRow, startCol, skelMass.rows(), skelMass.cols()) = skelMass;
            startRow+= skelMass.rows();
            startCol+= skelMass.cols();
        }
    }

    void ContactDynamic::initialize() {
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

            // Use these to construct the mass matrix and external forces vector.
            rows += skel->getMassMatrix().rows();
            cols += skel->getMassMatrix().cols();
        }

        mConstrForces.resize(getNumSkels());
        for (int i = 0; i < getNumSkels(); i++)
            mConstrForces[i] = VectorXd::Zero(mSkels[i]->getNumDofs());

        mM = MatrixXd::Zero(rows, cols);
        mTauStar = VectorXd::Zero(rows);

        // Construct mass matrix/external forces (tau)
        // Precomputed here because it is easier to get the matrix dimensions here.
        /*        int startRow = 0;
        int startCol = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            MatrixXd skelMass = skel->getMassMatrix();
            mM.block(startRow, startCol, skelMass.rows(), skelMass.cols()) = skelMass;
            startRow+= skelMass.rows();
            startCol+= skelMass.cols();
            }*/

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
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
    }

    void ContactDynamic::destroy() {
        if (mCollision) {
            delete mCollision;
        }
    }

    void ContactDynamic::applyContactForces() {
        mCollision->clearAllContacts();
        mCollision->checkCollision(false);
        if (getNumContacts() == 0)
            return;

        cleanupContact();
        fillMatrices();
        solve();
        applySolution();

    }
    
    void ContactDynamic::fillMatrices() {
        int c = getNumContacts();
        // cout << " contact # = " << c << endl;
        computeMassMat();
        computeTauStar();
        
        MatrixXd N = getNormalMatrix();
        MatrixXd B = getBasisMatrix();
        MatrixXd E = getContactMatrix();
        MatrixXd Minv = mM.inverse();
        MatrixXd mu = getMuMatrix();
        
        assert(Minv.rows() == Minv.cols());
        assert(Minv.cols() == N.rows());
        assert(Minv.cols() == B.rows());
        assert(N.cols() == E.cols());
        assert(B.cols() == E.rows());
        assert(mu.rows() == mu.cols());

        int dimA = c * (2 + mNumDir); // dimension of A is c + cd + c

        // Construct the intermediary blocks.
        MatrixXd Ntranspose = N.transpose();
        MatrixXd Btranspose = B.transpose();
        MatrixXd nTmInv = Ntranspose * Minv;
        MatrixXd bTmInv = Btranspose * Minv;
                
        // Construct
        //int cd = c * mNumDir;
        dimA = c;
        mA = MatrixXd::Zero(dimA, dimA);
        mA.block(0, 0, c, c) = nTmInv * N;
        /*        mA.block(0, c, c, cd) = nTmInv * B;
        mA.block(c, 0, cd, c) = bTmInv * N;
        mA.block(c, c, cd, cd) = bTmInv * B;
        mA.block(c, c + cd, cd, c) = E / (mDt*mDt);
        mA.block(c + cd, 0, c, c) = mu;
        mA.block(c + cd, c, c, cd) = -E.transpose();
        */
        // Construct Q
        mQBar = VectorXd::Zero(dimA);
        mQBar.block(0, 0, c, 1) = nTmInv * mTauStar;
        // mQBar.block(c, 0, cd, 1) = bTmInv * mTauStar;

        //mQBar /= mDt;
        /*
        MatrixXd mat = mTauStar;
        for(int i = 0; i < mat.rows(); i++)
            for(int j = 0; j < mat.cols(); j++)
                if(mat(i, j) > 1e-6 || mat(i,j) < -1e-6)
                    cout <<"mat[" << i << "][" << j << "]= " << mat(i,j) << endl;
        */
    }

    bool ContactDynamic::solve() {
        lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
        bool b = solver.Solve(mA, mQBar, mX);
        return b;
    }
    
    void ContactDynamic::applySolution() {
        int c = getNumContacts();

        // First compute the external forces
        int mStar = mM.rows(); // a hacky way to get the dimension
        VectorXd forces(VectorXd::Zero(mStar));
        VectorXd f_n = mX.block(0, 0, c, 1);
        //VectorXd f_d = mX.block(c, 0, c * mNumDir, 1);

        // Note, we need to un-scale by dt (was premultiplied in).
        // If this turns out to be a perf issue (above, as well as recomputing matrices 
        // again), consider caching both to improve performance.
        //MatrixXd N = getNormalMatrix();
        //MatrixXd B = getBasisMatrix();
        MatrixXd N = getNormalMatrix() / mDt;
        //        MatrixXd B = getBasisMatrix() / mDt;
        //forces = (N * f_n) + (B * f_d);
        forces = N * f_n;
        // Next, apply the external forces skeleton by skeleton.
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            int nDof = mSkels[i]->getNumDofs();
            mConstrForces[i] = forces.block(startRow, 0, nDof, 1); 
            startRow += nDof;
        }   
    }
    /*
    void ContactDynamic::applyExternalForces() {
        //        int c = getNumContacts() * 3;
        int c = getNumContacts();

        // First compute the external forces
        int mStar = mM.rows(); // a hacky way to get the dimension
        VectorXd forces(VectorXd::Zero(mStar));
        VectorXd f_n = mX.block(0, 0, c, 1);
        VectorXd f_d = mX.block(c, 0, c * mNumDir, 1);

        // Note, we need to un-scale by dt (was premultiplied in).
        // If this turns out to be a perf issue (above, as well as recomputing matrices 
        // again), consider caching both to improve performance.
        MatrixXd N = getNormalMatrix() / mDt;
        MatrixXd B = getBasisMatrix() / mDt;
        forces = (N * f_n) + (B * f_d);
        // Next, apply the external forces skeleton by skeleton.
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            int numRows = skel->getNumDofs();
            VectorXd extForces = forces.block(startRow, 0, numRows, 1);
            skel->applyAdditionalExternalForces(extForces);
            startRow += numRows;
        }
        
    }
*/
    MatrixXd ContactDynamic::getJacobian(kinematics::BodyNode* node, const Vector3d& p) const {
        const int nDofs = node->getSkel()->getNumDofs();
        MatrixXd J( MatrixXd::Zero(3, nDofs) );

        for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++) {
            int i = node->getDependentDof(dofIndex);
            VectorXd Jcol = utils::xformHom(node->getDerivWorldTransform(dofIndex), p);
            J.col(i) = Jcol;
        }
        return J;
    }

    MatrixXd ContactDynamic::getNormalMatrix() const {
        MatrixXd N(MatrixXd::Zero(getNumTotalDofs(), getNumContacts()));

        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
            int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
            int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

            int index1 = mIndices[skelID1];
            int NDOF1 = c.bd1->getSkel()->getNumDofs();
            int index2 = mIndices[skelID2];
            int NDOF2 = c.bd2->getSkel()->getNumDofs();

            // NOTE: indices shoudl be verified
            Vector3d p = c.point;
            Vector3d N21 = c.normal;
            Vector3d N12 = -c.normal;
            MatrixXd J21 = getJacobian(c.bd1, p);
            MatrixXd J12 = getJacobian(c.bd2, p);

            N.block(index1, i, NDOF1, 1) = J21.transpose() * N21;
            N.block(index2, i, NDOF2, 1) = J12.transpose() * N12;
        }

        N = N * mDt;
        
        return N;
    }

    MatrixXd ContactDynamic::getTangentBasisMatrix(const Vector3d& p, const Vector3d& n) const {
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

    MatrixXd ContactDynamic::getBasisMatrix() const {
        MatrixXd B(MatrixXd::Zero(getNumTotalDofs(), getNumContacts() * getNumContactDirections()));

        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
            int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
            int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

            int index1 = mIndices[skelID1];
            int NDOF1 = c.bd1->getSkel()->getNumDofs();
            int index2 = mIndices[skelID2];
            int NDOF2 = c.bd2->getSkel()->getNumDofs();


            // NOTE: indices shoud be verified
            Vector3d p = c.point;
            MatrixXd B21 = getTangentBasisMatrix(p, -c.normal);
            MatrixXd B12 = getTangentBasisMatrix(p, c.normal);
            MatrixXd J21 = getJacobian(c.bd1, p);
            MatrixXd J12 = getJacobian(c.bd2, p);

            B.block(index1, i * getNumContactDirections(), NDOF1, getNumContactDirections()) = J21.transpose() * B21;
            B.block(index2, i * getNumContactDirections(), NDOF2, getNumContactDirections()) = J12.transpose() * B12;
        }

        B = B * mDt;
        
        return B;
    }

    MatrixXd ContactDynamic::getContactMatrix() const {
        int numDir = getNumContactDirections();
        MatrixXd E(
            MatrixXd::Zero(getNumContacts() * numDir, getNumContacts())
            );
        MatrixXd column(MatrixXd::Ones(numDir, 1));            
        for (int i = 0; i < getNumContacts(); i++) {
            E.block(i * numDir, i, numDir, 1) = column;
        }
        return E;
    }

    MatrixXd ContactDynamic::getMuMatrix() const {
        int c = getNumContacts();
        return (MatrixXd::Identity(c, c) * mMu);

    }

    int ContactDynamic::getNumContacts() const { 
        return mCollision->getNumContact(); 
    }

    void ContactDynamic::cleanupContact() {
        vector<int> deleteIDs;
        for (int i = 0; i < getNumContacts(); i++) {
            ContactPoint& c = mCollision->getContact(i);
            bool isUnique = true;
            for (unsigned int j = 0; j < i; j++) {
                if(c.point != mCollision->getContact(j).point){
                    continue;
                }else{
                    deleteIDs.push_back(i);
                    break;
                }
            }            
        }
        if (deleteIDs.size() == 0)
            return;
        for (int i = deleteIDs.size() - 1; i >= 0; i--) {
            mCollision->mContactPointList.erase(mCollision->mContactPointList.begin() + deleteIDs[i]);
        }
    }
} // namespace dynamics
/*
            collision_checking::ContactPoint& c = mCollision->getContact(i);
            int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
            int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

            int index1 = mIndices[skelID1];
            int NDOF1 = c.bd1->getSkel()->getNumDofs();
            int index2 = mIndices[skelID2];
            int NDOF2 = c.bd2->getSkel()->getNumDofs();

            // NOTE: indices shoudl be verified
            Vector3d p = c.point;
            Vector3d N21 = -c.normal;
            Vector3d N12 = c.normal;
            MatrixXd J21 = getJacobian(c.bd1, p);
            MatrixXd J12 = getJacobian(c.bd2, p);
*/
