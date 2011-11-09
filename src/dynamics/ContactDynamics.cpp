#include "ContactDynamics.h"

#include "kinematics/BodyNode.h"
#include "lcpsolver/LCPSolver.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/collision_skeleton.h"
#include "utils/UtilsMath.h"

namespace dynamics {
    ContactDynamic::ContactDynamic(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu, int _d)
        : mSkels(_skels), mDt(_dt), mMu(_mu), mNumDir(_d), mCollision(NULL) {
        initialize();
    }

    ContactDynamic::~ContactDynamic() {
        destroy();
    }

    // Helper function to compute the T* vector an individual skeleton
    // Used when computing the global T* vector
    static Eigen::VectorXd getSkelTauStar(SkeletonDynamics* skel, const double dt) {
        Eigen::VectorXd tau = skel->getExternalForces();
        Eigen::VectorXd tauStar(Eigen::VectorXd::Zero(tau.rows()));
        tauStar = (skel->getMassMatrix() * skel->getQDotVector()) * (dt * (skel->getCoriolisVector() - tau));
        return tauStar;
    }

    void ContactDynamic::initialize() {
        // Allocate the Collision Detection class
        mCollision = new collision_checking::SkeletonCollision();

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

        mM = Eigen::MatrixXd::Zero(rows, cols);
        mTauStar = Eigen::VectorXd::Zero(rows);

        // Construct mass matrix/external forces (tau)
        // Precomputed here because it is easier to get the matrix dimensions here.
        int startRow = 0;
        int startCol = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            Eigen::MatrixXd skelMass = skel->getMassMatrix();
            Eigen::VectorXd skelTau = getSkelTauStar(skel, mDt);
            assert(skelMass.rows() == skelTau.rows());

            mM.block(startRow, startCol, skelMass.rows(), skelMass.cols()) = skelMass;
            mTauStar.block(startRow, 0, skelTau.rows(), 1) = skelTau;

            startRow+= skelMass.rows();
            startCol+= skelMass.cols();
        }

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

        fillMatrices();
        solve();
        applyExternalForces();
    }
    
    void ContactDynamic::fillMatrices() {
        int c = getNumContacts() * 3; // each contact is associated with 3 values

        // Retrieve all of the matrices
        // TODO, should assert the dimensions of the matrices
        Eigen::MatrixXd N = getNormalMatrix();
        Eigen::MatrixXd B = getBasisMatrix();
        Eigen::MatrixXd E = getContactMatrix();
        Eigen::MatrixXd Minv = getMassMatrix().inverse();
        Eigen::MatrixXd mu = getMuMatrix();
        Eigen::VectorXd tauStar = getTauStarVector();

        assert(Minv.rows() == Minv.cols());
        assert(Minv.cols() == N.rows());
        assert(Minv.cols() == B.rows());
        assert(N.cols() == E.cols() == c);
        assert(B.cols() == E.rows() == (c * mNumDir));
        assert(mu.rows() == mu.cols() == c);

        int dimA = c * (2 + mNumDir); // dimension of A is c + cd + c

        // Construct the intermediary blocks.
        Eigen::MatrixXd Ntranspose = N.transpose();
        Eigen::MatrixXd Btranspose = B.transpose();
        Eigen::MatrixXd nTmInv = Ntranspose * Minv;
        Eigen::MatrixXd bTmInv = Btranspose * Minv;

        // Construct A
        mA = Eigen::MatrixXd::Zero(dimA, dimA);
        mA.block(0, 0, c, c) = nTmInv * N;
        mA.block(0, c, c, c * mNumDir) = nTmInv * B;
        mA.block(c, 0, c * mNumDir, c) = bTmInv * N;
        mA.block(c, c, c * mNumDir, c * mNumDir) = bTmInv * B;
        mA.block(c, c + (c * mNumDir), E.rows(), E.cols()) = E;
        mA.block(c + (c * mNumDir), 0, mu.rows(), mu.cols()) = mu;
        mA.block(c + (c * mNumDir), c, E.cols(), E.rows()) = -E.transpose();

        // Construct Q
        mQBar = Eigen::VectorXd::Zero(dimA);
        mQBar.block(0, 0, c, 1) = nTmInv * tauStar;
        mQBar.block(c, 0, c * mNumDir, 1) = bTmInv * tauStar;
    }

    bool ContactDynamic::solve() {
        lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
        return solver.Solve(mA, mQBar, mX);
    }
    
    void ContactDynamic::applyExternalForces() {
        int c = getNumContacts() * 3;

        // First compute the external forces
        int mStar = mM.rows(); // a hacky way to get the dimension
        Eigen::VectorXd forces(Eigen::VectorXd::Zero(mStar));
        Eigen::VectorXd f_n = mX.block(0, 0, c, 1);
        Eigen::VectorXd f_d = mX.block(c, 0, c * mNumDir, 1);

        // Note, we need to un-scale by dt (was premultiplied in).
        // If this turns out to be a perf issue (above, as well as recomputing matrices 
        // again), consider caching both to improve peformance.
        Eigen::MatrixXd N = getNormalMatrix() / mDt;
        Eigen::MatrixXd B = getBasisMatrix() / mDt;
        forces = (N * f_n) + (B * f_d);

        // Next, apply the external forces skeleton by skeleton.
        int startRow = 0;
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            int numRows = skel->getNumDofs();
            Eigen::VectorXd extForces = forces.block(startRow, 0, numRows, 1);
            skel->applyAdditionalExternalForces(extForces);
            startRow += numRows;
        }
        
    }

    Eigen::MatrixXd ContactDynamic::getJacobian(kinematics::BodyNode* node, const Eigen::Vector3d& p) const {
        const int nDofs = node->getSkel()->getNumDofs();
        Eigen::MatrixXd J( Eigen::MatrixXd::Zero(3, nDofs) );

        for(int dofIndex = 0; dofIndex < node->getNumDependentDofs(); dofIndex++) {
            int i = node->getDependentDof(dofIndex);
            Eigen::VectorXd Jcol = utils::xformHom(node->getDerivWorldTransform(dofIndex), p);
            J.col(i) = Jcol;
        }
        return J;
    }

    /*
      struct ContactPoint {
      Eigen::Vector3d point;
      Eigen::Vector3d normal;
      kinematics::BodyNode *bd1;
      kinematics::BodyNode *bd2;
      };
    */
    Eigen::MatrixXd ContactDynamic::getNormalMatrix() const {
        Eigen::MatrixXd N(
            Eigen::MatrixXd::Zero(getNumTotalDofs(), getNumContacts())
            );

        for (int i = 0; i < getNumContacts(); i++) {
            collision_checking::ContactPoint& c = mCollision->getContact(i);
            int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
            int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

            int index1 = mIndices[skelID1];
            int NDOF1 = c.bd1->getSkel()->getNumDofs();
            int index2 = mIndices[skelID1];
            int NDOF2 = c.bd2->getSkel()->getNumDofs();

            // NOTE: indices shoudl be verified
            Eigen::Vector3d p = c.point;
            Eigen::Vector3d N21 = -c.normal;
            Eigen::Vector3d N12 = c.normal;
            Eigen::MatrixXd J21 = getJacobian(c.bd1, p);
            Eigen::MatrixXd J12 = getJacobian(c.bd2, p);

            N.block(index1, i, NDOF1, 1) = J21.transpose() * N21;
            N.block(index2, i, NDOF2, 1) = J12.transpose() * N12;
        }

        N = N * mDt;
        
        return N;
    }

    Eigen::MatrixXd ContactDynamic::getTangentBasisMatrix(const Vector3d& p, const Vector3d& n) const {
        Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, mNumDir));

        // Pick an arbitrary vector to take the cross product of (in this case, Z-axis)
        Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(n);
        // If they're too close, pick another tangent (use X-axis as arbitrary vector)
        if (tangent.norm() < EPSILON) {
            tangent = Eigen::Vector3d::UnitX().cross(n);
        }
        // Do we need to normalize the tangent here?
        tangent.normalize();

        // Rotate the tangent around the normal to compute bases.
        // Note: a possible speedup is in place for mNumDir % 2 = 0 
        // Each basis and its opposite belong in the matrix, so we iterate half as many times.
        double angle = (2 * M_PI) / mNumDir;
        int iter = (mNumDir % 2 == 0) ? mNumDir / 2 : mNumDir;
        for (int i = 0; i < iter; i++) {
            Eigen::Vector3d basis = Eigen::Quaterniond(Eigen::AngleAxisd(i * angle, n)) * tangent;
            T.block(0, i, 3, 1) = basis;

            if (mNumDir % 2 == 0) {
                T.block(0, i + iter, 3, 1) = -basis;
            }
        }

        return T;
    }

    Eigen::MatrixXd ContactDynamic::getBasisMatrix() const {
        Eigen::MatrixXd B(
            Eigen::MatrixXd::Zero(getNumTotalDofs(), getNumContacts())
            );

        for (int i = 0; i < getNumContacts(); i++) {
            collision_checking::ContactPoint& c = mCollision->getContact(i);
            int skelID1 = mBodyIndexToSkelIndex[c.bdID1];
            int skelID2 = mBodyIndexToSkelIndex[c.bdID2];

            int index1 = mIndices[skelID1];
            int NDOF1 = c.bd1->getSkel()->getNumDofs();
            int index2 = mIndices[skelID1];
            int NDOF2 = c.bd2->getSkel()->getNumDofs();

            // NOTE: indices shoud be verified
            Eigen::Vector3d p = c.point;
            Eigen::MatrixXd B21 = getTangentBasisMatrix(p, -c.normal);
            Eigen::MatrixXd B12 = getTangentBasisMatrix(p, c.normal);
            Eigen::MatrixXd J21 = getJacobian(c.bd1, p);
            Eigen::MatrixXd J12 = getJacobian(c.bd2, p);

            B.block(index1, i, NDOF1, getNumContactDirections()) = J21.transpose() * B21;
            B.block(index2, i, NDOF2, getNumContactDirections()) = J12.transpose() * B12;
        }

        B = B * mDt;
        
        return B;
    }

    Eigen::MatrixXd ContactDynamic::getContactMatrix() const {
        int numDir = getNumContactDirections();
        Eigen::MatrixXd E(
            Eigen::MatrixXd::Zero(getNumContacts() * numDir, getNumContacts())
            );
        Eigen::MatrixXd column(Eigen::MatrixXd::Ones(numDir, 1));            
        for (int i = 0; i < getNumContacts(); i++) {
            E.block(i * numDir, i, numDir, 1) = column;
        }
        return E;
    }

    Eigen::MatrixXd ContactDynamic::getMuMatrix() const {
        int c = getNumContacts() * 3;
        return (Eigen::MatrixXd::Identity(c, c) * mMu);

    }

    int ContactDynamic::getNumContacts() const { 
        return mCollision->getNumContact(); 
    }
} // namespace dynamics
