#include "JTController.h"
#include "kinematics/BodyNode.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

JTController::JTController(SkeletonDynamics* _skel, BodyNode* _body) : Controller(_skel) {
    mVirtualForce.setZero();
    mLocalPoint.setZero();
    mBodyNode = _body;
    mFrame = 0;
    
    mVirtualForce[1] = -1900.0;
    mLocalPoint = mBodyNode->getLocalCOM();

}

void JTController::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    mTorques.setZero();

    if (mFrame > 600 && mFrame < 800) {
        MatrixXd jacobian = mSkel->getJacobian(mBodyNode, mLocalPoint);
        mTorques = jacobian.transpose() * mVirtualForce;
    }
    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}
