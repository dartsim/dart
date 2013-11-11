#include "JTController.h"
#include "kinematics/BodyNode.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

JTController::JTController(SkeletonDynamics* _skel) : Controller(_skel) {
    mVirtualForce.setZero();
    mLocalPoint.setZero();
    mBodyNode = NULL;
}

void JTController::computeTorques() {
    MatrixXd jacobian = mSkel->getJacobian(mBodyNode, mLocalPoint);
    mTorques = jacobian.transpose() * mVirtualForce;

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;
}
