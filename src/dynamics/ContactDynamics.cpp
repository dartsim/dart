#include "ContactDynamics.h"

#include "kinematics/BodyNode.h"

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"

#include "collision/collision_skeleton.h"
#include "utils/UtilsMath.h"

namespace dynamics {
    ContactDynamic::ContactDynamic(const std::vector<SkeletonDynamics*>& _skels, double _dt, double _mu)
        : mSkels(_skels), mDt(_dt), mMu(_mu), mCollision(NULL) {
        initialize();
    }

    ContactDynamic::~ContactDynamic() {
        destroy();
    }

    void ContactDynamic::initialize() {
        // Allocate the Collision Detection class
        mCollision = new collision_checking::SkeletonCollision();

        mBodyIndexToSkelIndex.clear();
        // Add all body nodes into mCollision
        for (int i = 0; i < getNumSkels(); i++) {
            SkeletonDynamics* skel = mSkels[i];
            int nNodes = skel->getNumNodes();

            for (int j = 0; j < nNodes; j++) {
                kinematics::BodyNode* node = skel->getNode(j);
                mCollision->addCollisionSkeletonNode(node);
                mBodyIndexToSkelIndex.push_back(i);
            }
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
        Eigen::MatrixXd N = getNormalMatirx();
    }

    void ContactDynamic::solve() {
        
    }
    
    void ContactDynamic::applyExternalForces() {
        
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
    Eigen::MatrixXd ContactDynamic::getNormalMatirx() const {
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

    int ContactDynamic::getNumContacts() const {
        return mCollision->getNumContact();
    }


} // namespace dynamics
