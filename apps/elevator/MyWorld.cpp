#include "MyWorld.h"
#include "dynamics/ConstraintDynamics.h"
#include "kinematics/BodyNode.h"

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
}
