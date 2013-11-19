#include "MyWorld.h"
#include "dynamics/ConstraintDynamics.h"
#include "kinematics/BodyNode.h"
#include <Eigen/SVD>

using namespace kinematics;
using namespace Eigen;

#define FORCE_THRESHOLD 2500

void MyWorld::controlWalls() {
    Vector4d qBar;
    qBar << 1.57, 1.57, -1.57, -1.57;
    int nDof = getSkeleton(1)->getNumDofs();
    VectorXd torque(nDof);
    torque.setZero();
    for (int i = 6; i < nDof; i++)
        torque[i] = -mWallMaterial * (getSkeleton(1)->getPose()[i] - qBar[i - 6]) - 10 * getSkeleton(1)->getPoseVelocity()[i];
    getSkeleton(1)->setInternalForces(torque);
}

void MyWorld::computeImpact() {
    int nContact = getCollisionHandle()->getCollisionChecker()->getNumContacts();
    for (int i = 0; i < nContact; i++) {
        BodyNode* bd1 = getCollisionHandle()->getCollisionChecker()->getContact(i).collisionNode1->getBodyNode();
        BodyNode* bd2 = getCollisionHandle()->getCollisionChecker()->getContact(i).collisionNode2->getBodyNode();
        if (bd1->getSkel() == getSkeleton(0) || bd2->getSkel() == getSkeleton(0)) {
            Vector3d force = getCollisionHandle()->getCollisionChecker()->getContact(i).force;
            double forceMag = force.norm();
            if (forceMag > FORCE_THRESHOLD)
                mAccumulatedImpact += forceMag;
        }
    }
    // compute constraint force due to grasping
    double constraintForce = 0.0;
    int nConstr = getCollisionHandle()->getNumConstraints();
    for (int i = nConstr - 1; i >= 0; i--) {
        double f = getCollisionHandle()->getConstraint(i)->getLagrangeMultipliers().norm();
        //        cout << f << endl;
        if (f > 100000) {
            // if the constraint force exceeds 10 times of graviational force, break the constraints
            cout << "Break!" << endl;
            getCollisionHandle()->deleteConstraint(getCollisionHandle()->getConstraint(i));
        }
    }
}

void MyWorld::computeJointStress() {
    /*    int nDof = getSkeleton(0)->getNumDofs();
    int nNode = getSkeleton(0)->getNumNodes();
    MatrixXd J(nDof, 6 * nNode);
    J.setZero();
   
    for (int i = 0; i < nNode; i++) {
        MatrixXd linearJ = getSkeleton(0)->getNode(i)->getJacobianLinear().transpose();
        MatrixXd angularJ = getSkeleton(0)->getNode(i)->getJacobianAngular().transpose();

        for (int j = 0; j < getSkeleton(0)->getNode(i)->getNumDependentDofs(); j++) {
            int index = getSkeleton(0)->getNode(i)->getDependentDof(j);
            J.block(index, i * 6, 1, 3) = linearJ.row(j); 
            J.block(index, i * 6 + 3, 1, 3) = angularJ.row(j); 
        }
        //        J.block(0, i * 6, nDof, 3) = getSkeleton(0)->getNode(i)->getJacobianLinear().transpose();
        //        J.block(0, i * 6 + 3, nDof, 3) = getSkeleton(0)->getNode(i)->getJacobianAngular().transpose();
    }

    // Run SVD    
    JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeFullV);
    //cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    MatrixXd nullJ = svd.matrixV().rightCols(J.cols() - J.rows()).transpose();
    cout << nullJ.col(10).norm() << endl;

    //    cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    // cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
    */    
}

