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
    
    mVirtualForce[1] = -1000.0;
    mLocalPoint = mBodyNode->getLocalCOM();
    /*   double crouchPose[] = {-0.0166711, -0.250428, 0.0, 0.278799, 0.0, 0.0, 0.572453, 0.0, 0.0, -1.80614, 0.523637 ,0.0, 0.403953, 0.572453, 0.0, 0.0, -1.80614, 0.523637, 0.0, 0.403953, 0.0, -0.298531, 0.0, -0.0711807, 0.0, -0.227518, 0.421909, -0.0693028, -0.0821567, 0.00587502, 0.273915, 0.227518, 0.421909, 0.0693028, 0.0821567, 0.00587502, -0.273915};
     */

}

void JTController::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    if (mFrame > 700 && mFrame < 1000) {
        MatrixXd jacobian = mSkel->getJacobian(mBodyNode, mLocalPoint);
        mTorques = jacobian.transpose() * mVirtualForce;

        jacobian = mSkel->getJacobian(mSkel->getNode("fullbody1_h_heel_right"), mLocalPoint);
        mTorques += jacobian.transpose() * mVirtualForce;
    }

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++)
        mTorques[i] = 0.0;

    mFrame++;
}
